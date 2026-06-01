#!/usr/bin/env python3
"""
Serial Bridge Node (TX 패스스루 + RX odom 파싱 + 자동 재연결)
- TX: /fsm_cmd (std_msgs/String, JSON) -> ESP32 UART
- RX: ESP32 가 UART 로 보내는 {"T":"odom",...} 라인을 파싱해
      nav_msgs/Odometry(/odom) 발행 + TF(odom->base_link) 브로드캐스트.
      그 외 라인(pong 등)은 버림.
- EIO 등 포트 사망 시 자동으로 닫고 재연결 (에러 도배 방지)

흐름: fsm_node --/fsm_cmd--> [bridge_node] --UART--> ESP32(로컬 PID + odom 계산)
      ESP32 --UART(odom json)--> [bridge_node] --/odom + TF--> (Nav2/EKF/RViz...)

버전: v0.6 (odom RX 추가: parse -> /odom + TF)
"""

import os
import csv
import json
import math
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import serial


SERIAL_PORT = '/dev/ttyAMA0'
SERIAL_BAUD = 115200
MOTOR_CMD_TOPIC = '/fsm_cmd'    # ★ fsm_node 가 발행하는 토픽과 반드시 일치
ODOM_TOPIC  = '/odom'
ODOM_FRAME  = 'odom'
BASE_FRAME  = 'base_link'

STAT_PERIOD_SEC = 0.5
RX_POLL_SEC     = 0.02    # odom 10Hz 수신 -> 50Hz 폴링이면 여유 (라인 파싱)
RECONNECT_SEC   = 1.0
LOG_DIR = os.path.expanduser('~/bridge_logs')


class SerialBridge(Node):
    def __init__(self):
        super().__init__('bridge_node')

        self.ser = None
        self.connected = False
        self.rx_buffer = b''        # 라인 조립용 버퍼

        # CSV 로그 (DEBUG 레벨일 때만)
        self.debug_mode = (
            self.get_logger().get_effective_level() <= LoggingSeverity.DEBUG
        )
        self.csv_file = None
        self.csv_writer = None
        if self.debug_mode:
            self._open_csv_log()

        # 구독 (TX 패스스루)
        self.cmd_sub = self.create_subscription(
            String, MOTOR_CMD_TOPIC, self.on_motor_cmd, 10)

        # 발행 (RX odom)
        self.odom_pub = self.create_publisher(Odometry, ODOM_TOPIC, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # 통계
        self.tx_count = 0
        self.tx_count_window = 0
        self.rx_odom_count = 0
        self.rx_odom_window = 0
        self.last_payload = ''
        self.err_count = 0

        # 시리얼 최초 오픈 (실패해도 재연결 타이머가 계속 시도)
        if not self._open_serial():
            self.get_logger().error(
                f'[FAIL] serial open at startup: {SERIAL_PORT} (재연결 대기)')

        # 타이머
        self.rx_timer = self.create_timer(RX_POLL_SEC, self.on_rx_timer)
        self.reconnect_timer = self.create_timer(RECONNECT_SEC, self.on_reconnect_timer)
        self.stat_timer = self.create_timer(STAT_PERIOD_SEC, self.on_stat_timer)

        self.get_logger().info(
            f'[READY] bridge_node (debug={self.debug_mode}, '
            f'sub={MOTOR_CMD_TOPIC}, pub={ODOM_TOPIC})')

    # ---- 시리얼 열기 ----
    def _open_serial(self):
        try:
            self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            self.connected = True
            self.rx_buffer = b''
            self.get_logger().info(f'[OPEN] serial: {SERIAL_PORT} @ {SERIAL_BAUD}')
            return True
        except (serial.SerialException, OSError):
            self.ser = None
            self.connected = False
            return False

    # ---- 끊김 처리 (1회만 로그) ----
    def _handle_disconnect(self, where, e):
        was_connected = self.connected
        self.connected = False
        self.rx_buffer = b''
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
            self.ser = None
        if was_connected:
            self.err_count += 1
            self.get_logger().error(f'[DISCONNECT] {where}: {e} -> 자동 재연결 시도')
            self._csv_log('ERROR', f'{where}: {e}')

    # ---- 재연결 타이머 (1Hz) ----
    def on_reconnect_timer(self):
        if self.ser is not None:
            return
        if self._open_serial():
            self.get_logger().info('[RECONNECT] serial 재연결 성공')

    # ---- CSV ----
    def _open_csv_log(self):
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            fname = datetime.now().strftime('bridge_%Y%m%d_%H%M%S.csv')
            fpath = os.path.join(LOG_DIR, fname)
            self.csv_file = open(fpath, 'w', newline='', buffering=1)
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['timestamp', 'direction', 'content'])
            self.get_logger().info(f'[LOG] csv: {fpath}')
        except Exception as e:
            self.get_logger().error(f'[FAIL] csv open: {e}')
            self.csv_file = None
            self.csv_writer = None

    def _csv_log(self, direction, content):
        if self.csv_writer is None:
            return
        try:
            self.csv_writer.writerow([f'{time.time():.6f}', direction, content])
        except Exception:
            pass

    # ---- TX: /fsm_cmd -> UART ----
    def on_motor_cmd(self, msg):
        if self.ser is None:
            return
        payload = msg.data.strip()
        if not payload:
            return
        self._csv_log('RX_TOPIC', payload)
        try:
            self.ser.write((payload + '\n').encode('ascii'))
            self.tx_count += 1
            self.tx_count_window += 1
            self.last_payload = payload
            self._csv_log('TX_SERIAL', payload)
        except (serial.SerialException, OSError) as e:
            self._handle_disconnect('write', e)

    # ---- RX: 라인 단위로 읽어 odom 파싱 ----
    def on_rx_timer(self):
        if self.ser is None:
            return
        try:
            n = self.ser.in_waiting
            if n:
                self.rx_buffer += self.ser.read(n)
                while b'\n' in self.rx_buffer:
                    raw, self.rx_buffer = self.rx_buffer.split(b'\n', 1)
                    self._process_line(raw.decode('ascii', errors='ignore'))
                # 줄바꿈 없이 폭주하면 방어적으로 비움
                if len(self.rx_buffer) > 4096:
                    self.rx_buffer = b''
        except (serial.SerialException, OSError) as e:
            self._handle_disconnect('read', e)

    def _process_line(self, line):
        line = line.strip()
        if not line:
            return
        try:
            data = json.loads(line)
        except (json.JSONDecodeError, ValueError):
            if self.debug_mode:
                self._csv_log('RX_DROP', line)
            return
        if data.get('T') == 'odom':
            self._publish_odom(data)
        elif self.debug_mode:
            self._csv_log('RX_OTHER', line)   # pong 등

    # ---- odom dict -> /odom + TF ----
    def _publish_odom(self, d):
        now = self.get_clock().now().to_msg()
        x  = float(d.get('x',  0.0))
        y  = float(d.get('y',  0.0))
        th = float(d.get('th', 0.0))
        vx = float(d.get('vx', 0.0))
        wz = float(d.get('wz', 0.0))

        qz = math.sin(th * 0.5)
        qw = math.cos(th * 0.5)

        # nav_msgs/Odometry
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id  = BASE_FRAME
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x  = vx
        odom.twist.twist.angular.z = wz
        # 대략적 공분산 (robot_localization 융합 시 실측 기반으로 조정)
        odom.pose.covariance[0]   = 0.01    # x
        odom.pose.covariance[7]   = 0.01    # y
        odom.pose.covariance[35]  = 0.05    # yaw
        odom.twist.covariance[0]  = 0.01    # vx
        odom.twist.covariance[35] = 0.05    # wz
        self.odom_pub.publish(odom)

        # TF odom -> base_link
        tf = TransformStamped()
        tf.header.stamp = now
        tf.header.frame_id = ODOM_FRAME
        tf.child_frame_id  = BASE_FRAME
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

        self.rx_odom_count += 1
        self.rx_odom_window += 1
        self._csv_log('RX_ODOM', f'x={x:.3f} y={y:.3f} th={th:.3f}')

    # ---- 통계 ----
    def on_stat_timer(self):
        if self.tx_count_window > 0 or self.rx_odom_window > 0 or not self.connected:
            self.get_logger().info(
                f'[STAT] conn={self.connected} '
                f'tx={self.tx_count} ({self.tx_count_window/STAT_PERIOD_SEC:.0f}/s) '
                f'odom={self.rx_odom_count} ({self.rx_odom_window/STAT_PERIOD_SEC:.0f}/s) '
                f'err={self.err_count} last_cmd={self.last_payload}')
        self.tx_count_window = 0
        self.rx_odom_window = 0

    def destroy_node(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        if self.csv_file is not None:
            try:
                self.csv_file.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
