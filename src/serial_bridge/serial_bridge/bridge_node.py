#!/usr/bin/env python3
"""
Serial Bridge Node (TX-only)
- ROS2 /motor_cmd (std_msgs/String, JSON) → ESP32 UART
- 패스스루 방식: FSM이 만든 JSON 문자열을 그대로 시리얼로 전송
- 로그 전략:
    * INFO : 0.5초마다 통계만 (조용)
    * DEBUG: 모든 송수신 메시지를 ~/bridge_logs/*.csv 에 저장
    * ERROR: 시리얼 쓰기 실패 등은 즉시 출력 + CSV 기록

토픽 규격:
    /motor_cmd (std_msgs/String)
        주행:    {"T":"m","L":<-255~255>,"R":<-255~255>}
        비상정지: {"T":"e"}

작성: Eng A (ws)
버전: v0.2 (JSON 패스스루, RX 제거, CSV 로깅)
"""

import os
import csv
import time
from datetime import datetime

import rclpy
from rclpy.node import Node
from rclpy.logging import LoggingSeverity
from std_msgs.msg import String
import serial


# ─────────────────────────────────────────────────────────
# 설정 상수
# ─────────────────────────────────────────────────────────
SERIAL_PORT = '/dev/ttyAMA0'    # 실측 후 변경 가능
SERIAL_BAUD = 115200            # ESP32 펌웨어 기본값과 일치
MOTOR_CMD_TOPIC = '/motor_cmd'

STAT_PERIOD_SEC = 0.5           # 통계 출력 주기
LOG_DIR = os.path.expanduser('~/bridge_logs')


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # 1. 시리얼 포트 열기
        try:
            self.ser = serial.Serial(SERIAL_PORT, SERIAL_BAUD, timeout=0.1)
            self.get_logger().info(f'[OPEN] serial: {SERIAL_PORT} @ {SERIAL_BAUD}')
        except serial.SerialException as e:
            self.get_logger().error(f'[FAIL] serial open: {e}')
            self.ser = None

        # 2. CSV 로그 파일 준비 (DEBUG 레벨일 때만 실제 기록)
        self.debug_mode = (
            self.get_logger().get_effective_level() <= LoggingSeverity.DEBUG
        )
        self.csv_file = None
        self.csv_writer = None
        if self.debug_mode:
            self._open_csv_log()

        # 3. /motor_cmd 구독
        self.cmd_sub = self.create_subscription(
            String, MOTOR_CMD_TOPIC, self.on_motor_cmd, 10
        )

        # 4. 통계 카운터
        self.tx_count = 0           # 전체 누적 송신 수
        self.tx_count_window = 0    # 0.5초 윈도우 송신 수
        self.last_payload = ''      # 최근 송신 페이로드
        self.err_count = 0          # 누적 에러 수

        # 5. 통계 출력 타이머 (0.5초)
        self.stat_timer = self.create_timer(STAT_PERIOD_SEC, self.on_stat_timer)

        self.get_logger().info(
            f'[READY] serial_bridge running (debug={self.debug_mode}, '
            f'topic={MOTOR_CMD_TOPIC})'
        )

    # ─────────────────────────────────────────────────
    # CSV 로그 파일 열기
    # ─────────────────────────────────────────────────
    def _open_csv_log(self):
        try:
            os.makedirs(LOG_DIR, exist_ok=True)
            fname = datetime.now().strftime('bridge_%Y%m%d_%H%M%S.csv')
            fpath = os.path.join(LOG_DIR, fname)
            self.csv_file = open(fpath, 'w', newline='', buffering=1)  # line-buffered
            self.csv_writer = csv.writer(self.csv_file)
            self.csv_writer.writerow(['timestamp', 'direction', 'content'])
            self.get_logger().info(f'[LOG] csv: {fpath}')
        except Exception as e:
            self.get_logger().error(f'[FAIL] csv open: {e}')
            self.csv_file = None
            self.csv_writer = None

    # ─────────────────────────────────────────────────
    # CSV 한 줄 기록 (DEBUG 모드일 때만)
    # ─────────────────────────────────────────────────
    def _csv_log(self, direction: str, content: str):
        if self.csv_writer is None:
            return
        try:
            self.csv_writer.writerow([f'{time.time():.6f}', direction, content])
        except Exception as e:
            # 로그 실패는 동작에 영향 안 주도록 조용히 처리
            self.get_logger().warn(f'[CSV FAIL] {e}')

    # ─────────────────────────────────────────────────
    # /motor_cmd 콜백 (패스스루)
    # ─────────────────────────────────────────────────
    def on_motor_cmd(self, msg: String):
        if self.ser is None:
            return

        payload = msg.data.strip()
        if not payload:
            return  # 빈 문자열 무시

        # CSV: 토픽 수신 기록
        self._csv_log('RX_TOPIC', payload)

        # 시리얼 송신 (JSON + '\n')
        try:
            self.ser.write((payload + '\n').encode('ascii'))
            self.tx_count += 1
            self.tx_count_window += 1
            self.last_payload = payload
            # CSV: 시리얼 송신 기록
            self._csv_log('TX_SERIAL', payload)
        except serial.SerialException as e:
            self.err_count += 1
            self.get_logger().error(f'[FAIL] serial write: {e}')
            self._csv_log('ERROR', f'write_fail: {e}')

    # ─────────────────────────────────────────────────
    # 통계 출력 타이머 (0.5초마다)
    # ─────────────────────────────────────────────────
    def on_stat_timer(self):
        rate = self.tx_count_window / STAT_PERIOD_SEC  # msg/s
        if self.tx_count_window > 0:
            self.get_logger().info(
                f'[STAT] tx={self.tx_count} ({rate:.0f}msg/s) '
                f'err={self.err_count} last={self.last_payload}'
            )
        # 윈도우 카운터 리셋
        self.tx_count_window = 0

    # ─────────────────────────────────────────────────
    # 종료 처리
    # ─────────────────────────────────────────────────
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
