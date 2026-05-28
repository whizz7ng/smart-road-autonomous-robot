#!/usr/bin/env python3
"""
Serial Bridge Node
- ROS2 /cmd_vel ↔ ESP32 UART 단일문자 명령
- Pi 측: /dev/ttyUSB0 또는 /dev/ttyAMA0 (실측 시 결정)
- ESP32 측: 자체 펌웨어 (단일문자 명령: 'w','a','s','d','x','0~5','m')

작성: Eng A (ws)
버전: v0.1 (송신만, IMU 수신은 펌웨어 IMU 추가 후 구현)
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import threading


# ─────────────────────────────────────────────────────────
# 설정 상수
# ─────────────────────────────────────────────────────────
SERIAL_PORT = '/dev/ttyUSB0'    # 실측 후 변경 가능
SERIAL_BAUD = 115200            # ESP32 펌웨어 기본값과 일치
CMD_VEL_TOPIC = '/cmd_vel'

# cmd_vel 임계값 (이하는 정지로 처리)
LINEAR_DEAD_ZONE = 0.05         # m/s
ANGULAR_DEAD_ZONE = 0.1         # rad/s


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

        # 2. /cmd_vel 구독
        self.cmd_sub = self.create_subscription(
            Twist, CMD_VEL_TOPIC, self.on_cmd_vel, 10
        )

        # 3. 수신 스레드 (지금은 로그만, 나중에 IMU 파싱 추가)
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        self.last_cmd = ''
        self.get_logger().info('[READY] serial_bridge running')

    # ─────────────────────────────────────────────────
    # cmd_vel → 단일문자 변환
    # ─────────────────────────────────────────────────
    def twist_to_char(self, lx: float, az: float) -> str:
        """
        ESP32 펌웨어 명령 매핑:
          정지     → 'x'
          전진     → 'w' (PWM 80)
          후진     → 's' (PWM 80)
          좌회전   → 'a' (A=80, B=120)
          우회전   → 'd' (A=120, B=80)
        """
        # 데드존: 둘 다 작으면 정지
        if abs(lx) < LINEAR_DEAD_ZONE and abs(az) < ANGULAR_DEAD_ZONE:
            return 'x'

        # 각속도 우선 (회전 명령)
        if abs(az) >= ANGULAR_DEAD_ZONE:
            return 'a' if az > 0 else 'd'

        # 선속도
        return 'w' if lx > 0 else 's'

    # ─────────────────────────────────────────────────
    # /cmd_vel 콜백
    # ─────────────────────────────────────────────────
    def on_cmd_vel(self, msg: Twist):
        if self.ser is None:
            return

        cmd = self.twist_to_char(msg.linear.x, msg.angular.z)

        # 같은 명령 반복 송신 방지 (옵션)
        if cmd == self.last_cmd:
            return

        try:
            self.ser.write(cmd.encode('ascii'))
            self.get_logger().info(
                f'[TX] linear={msg.linear.x:.2f} angular={msg.angular.z:.2f} → "{cmd}"'
            )
            self.last_cmd = cmd
        except serial.SerialException as e:
            self.get_logger().error(f'[FAIL] write: {e}')

    # ─────────────────────────────────────────────────
    # 시리얼 수신 루프 (지금은 로그만)
    # ─────────────────────────────────────────────────
    def rx_loop(self):
        while rclpy.ok() and self.ser is not None:
            try:
                line = self.ser.readline().decode(errors='ignore').strip()
                if line:
                    self.get_logger().info(f'[RX] {line}')
                    # TODO: 펌웨어 IMU 추가 후 JSON 파싱 + /imu/data publish
            except Exception as e:
                self.get_logger().warn(f'[RX FAIL] {e}')

    def destroy_node(self):
        if self.ser is not None:
            self.ser.close()
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
