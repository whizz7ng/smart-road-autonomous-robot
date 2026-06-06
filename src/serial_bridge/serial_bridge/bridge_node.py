#!/usr/bin/env python3
"""
ROS2 bridge node: cmd_vel <-> ESP32 (UART)

흐름:
  /cmd_vel (geometry_msgs/Twist)
      -> 차동구동 역기구학으로 좌/우 목표 RPM 계산
      -> ESP32 (UART) 에 {"T":"v","L":..,"R":..}\n  10Hz 송신
         (ESP32 워치독 500ms 유지)

  ESP32 odo 텔레메트리 (UART, 50Hz)
      {"T":"odo","rl":..,"rr":..,"gz":..,...}\n
      -> 직진 v 는 바퀴 엔코더, heading(theta) 는 IMU 자이로(gz)로 적분
      -> 중점법으로 (x, y, theta) 적분
      -> /odom 퍼블리시 + TF odom -> base_link 브로드캐스트
"""

import math
import json
import threading
import time

import serial
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


# ===== 로봇 기하 =====
WHEEL_RADIUS = 0.0325   # m
WHEEL_BASE   = 0.255    # m

# ===== UART =====
SERIAL_PORT  = "/dev/ttyAMA0"   # 라즈파이 GPIO UART. USB-TTL 쓰면 /dev/ttyUSB0
SERIAL_BAUD  = 115200

# ===== 제어 =====
CMD_RATE_HZ = 10        # ESP32 워치독(500ms) 보호
CMD_STALE_S = 0.5       # cmd_vel 끊긴 후 자동 정지
ODOM_PUB_RATE_HZ = 20

# ===== IMU =====
# 자이로 yaw rate 부호. CCW(반시계) 회전 시 odom theta 가 증가해야 정상.
# 반대로 가면 -1.0 으로 바꿀 것.
GYRO_SIGN =1.0

# ===== 프레임 =====
ODOM_FRAME = "odom"
BASE_FRAME = "base_link"

# ===== 직진 trim (오른쪽 쏠림 보정) =====
WHEEL_TRIM_L = 1.0
WHEEL_TRIM_R = 1.08    # 오른쪽 쏠리면 이 값을 1.02, 1.04 ... 올려라

def twist_to_wheel_rpm(vx: float, wz: float):
    """linear m/s, angular rad/s -> (rpm_l, rpm_r)."""
    v_l = vx - wz * WHEEL_BASE / 2.0
    v_r = vx + wz * WHEEL_BASE / 2.0
    k = 60.0 / (2.0 * math.pi * WHEEL_RADIUS)
    return v_l * k * WHEEL_TRIM_L, v_r * k * WHEEL_TRIM_R


def wheel_rpm_to_twist(rpm_l: float, rpm_r: float):
    """(rpm_l, rpm_r) -> (v m/s, w rad/s)."""
    k = (2.0 * math.pi * WHEEL_RADIUS) / 60.0
    v_l = rpm_l * k
    v_r = rpm_r * k
    return (v_l + v_r) / 2.0, (v_r - v_l) / WHEEL_BASE


class BridgeNode(Node):
    def __init__(self):
        super().__init__("bridge_node")

        # ----- 파라미터 -----
        self.declare_parameter("serial_port", SERIAL_PORT)
        self.declare_parameter("serial_baud", SERIAL_BAUD)
        port = self.get_parameter("serial_port").value
        baud = int(self.get_parameter("serial_baud").value)
        # ----- UART -----
        try:
            self.ser = serial.Serial(port, baud, timeout=0.5)
            self.ser.reset_input_buffer()
        except serial.SerialException as e:
            self.get_logger().fatal(f"UART open failed ({port}): {e}")
            raise SystemExit(1)

        # ----- pub/sub -----
        self.create_subscription(Twist, "cmd_vel", self.cmd_vel_cb, 10)
        self.odom_pub = self.create_publisher(Odometry, "odom", 50)
        self.tf_br = TransformBroadcaster(self)

        # ----- 공유 상태 -----
        self.lock = threading.Lock()
        self.target_rpm_l = 0.0
        self.target_rpm_r = 0.0
        self.last_cmd_t = 0.0

        # odom 적분
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_odom_t = None
        self.last_v = 0.0
        self.last_w = 0.0

        # ----- 수신 스레드 + 송신 타이머 -----
        self.running = True
        self.rx_thread = threading.Thread(target=self.uart_recv_loop, daemon=True)
        self.rx_thread.start()

        self.create_timer(1.0 / CMD_RATE_HZ, self.send_cmd_timer)
        self.create_timer(1.0 / ODOM_PUB_RATE_HZ, self.publish_odom_timer)

        self.get_logger().info(
            f"bridge_node up. UART={port}@{baud} "
            f"wheel R={WHEEL_RADIUS:.4f}m base={WHEEL_BASE:.4f}m "
            f"(heading=gyro, sign={GYRO_SIGN:+.0f})")

    # ===== cmd_vel 콜백 =====
    def cmd_vel_cb(self, msg: Twist):
        rpm_l, rpm_r = twist_to_wheel_rpm(msg.linear.x, msg.angular.z)
        with self.lock:
            self.target_rpm_l = rpm_l
            self.target_rpm_r = rpm_r
            self.last_cmd_t = time.time()

    # ===== 주기 송신 (10Hz) =====
    def send_cmd_timer(self):
        with self.lock:
            l, r = self.target_rpm_l, self.target_rpm_r
            stale = (time.time() - self.last_cmd_t) > CMD_STALE_S
        if stale:
            l = r = 0.0
        payload = (json.dumps({"T": "v", "L": round(l, 2), "R": round(r, 2)}) + "\n").encode()
        try:
            with self.lock:
                self.ser.write(payload)
        except serial.SerialException as e:
            self.get_logger().warn(f"UART write failed: {e}")

    # ===== UART 수신 루프 (50Hz odo -> odom) =====
    def uart_recv_loop(self):
        while self.running:
            try:
                line = self.ser.readline().decode(errors="ignore").strip()
            except serial.SerialException as e:
                self.get_logger().warn(f"UART read error: {e}")
                time.sleep(0.1)
                continue

            if not line.startswith("{"):
                continue

            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue

            if d.get("T") != "odo":
                continue

            try:
                rl = float(d["rl"])
                rr = float(d["rr"])
                gz = float(d.get("gz", 0.0))   # deg/s (자이로 z, IMU)
            except (KeyError, TypeError, ValueError):
                continue
            self.update_odom(rl, rr, gz)

    # ===== 오도메트리 적분 =====

    def update_odom(self, rpm_l: float, rpm_r: float, gz: float):
        now = self.get_clock().now()
        if self.last_odom_t is None:
            self.last_odom_t = now
            return
        dt = (now - self.last_odom_t).nanoseconds * 1e-9
        self.last_odom_t = now
        if dt <= 0.0 or dt > 0.5:
            return

        # 직진속도 v 는 바퀴 엔코더, heading(w) 는 자이로 yaw rate 사용
        v, _w_wheel = wheel_rpm_to_twist(rpm_l, rpm_r)
        w = math.radians(gz) * GYRO_SIGN   # deg/s -> rad/s

        dtheta = w * dt
        mid_theta = self.theta + dtheta / 2.0
        with self.lock:
            self.x += v * math.cos(mid_theta) * dt
            self.y += v * math.sin(mid_theta) * dt
            self.theta = math.atan2(
                math.sin(self.theta + dtheta),
                math.cos(self.theta + dtheta))
            self.last_v = v
            self.last_w = w
        # publish_odom 호출 제거됨 (Timer 가 담당)

    def publish_odom_timer(self):
        """20Hz 균일 발행. UART 지터와 분리."""
        now = self.get_clock().now()
        with self.lock:
            v = self.last_v
            w = self.last_w
        self.publish_odom(now, v, w)


    def publish_odom(self, stamp, v: float, w: float):
        stamp_msg = stamp.to_msg()
        with self.lock:
            x = self.x
            y = self.y
            theta = self.theta
        qz = math.sin(theta / 2.0)
        qw = math.cos(theta / 2.0)

        # /odom
        odom = Odometry()
        odom.header.stamp = stamp_msg
        odom.header.frame_id = ODOM_FRAME
        odom.child_frame_id = BASE_FRAME
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        # TF odom -> base_link
        tf = TransformStamped()
        tf.header.stamp = stamp_msg
        tf.header.frame_id = ODOM_FRAME
        tf.child_frame_id = BASE_FRAME
        tf.transform.translation.x = x
        tf.transform.translation.y = y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_br.sendTransform(tf)

    # ===== 종료 =====
    def shutdown(self):
        self.running = False
        try:
            stop = (json.dumps({"T": "v", "L": 0.0, "R": 0.0}) + "\n").encode()
            self.ser.write(stop)
            time.sleep(0.05)
            self.ser.write((json.dumps({"T": "e"}) + "\n").encode())
        except Exception:
            pass
        finally:
            self.ser.close()


def main(args=None):
    rclpy.init(args=args)
    node = BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
