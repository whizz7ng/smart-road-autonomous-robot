#!/usr/bin/env python3
"""
Test Driving Node
- /motor_cmd 로 실험용 PWM 명령 전송
- 동작:
    1) 전진 3초  (L=120,  R=120)
    2) 후진 3초  (L=-120, R=-120)
    3) 비상정지 1회 전송 후 종료

토픽:
    /motor_cmd (std_msgs/String)

JSON 형식:
    {"T":"m","L":120,"R":120}
    {"T":"e"}
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


MOTOR_CMD_TOPIC = '/motor_cmd'


class DrivingNode(Node):
    def __init__(self):
        super().__init__('driving_node')

        self.pub = self.create_publisher(String, MOTOR_CMD_TOPIC, 10)

        self.get_logger().info('[READY] driving_node started')

        # 노드 시작 후 바로 테스트 실행
        self.run_test()

    # ─────────────────────────────────────────────
    # JSON publish helper
    # ─────────────────────────────────────────────
    def publish_motor(self, left_pwm: int, right_pwm: int):
        payload = {
            "T": "m",
            "L": left_pwm,
            "R": right_pwm
        }

        msg = String()
        msg.data = json.dumps(payload)

        self.pub.publish(msg)

        self.get_logger().info(f'[PUB] {msg.data}')

    # ─────────────────────────────────────────────
    # Emergency stop
    # ─────────────────────────────────────────────
    def publish_estop(self):
        payload = {
            "T": "e"
        }

        msg = String()
        msg.data = json.dumps(payload)

        self.pub.publish(msg)

        self.get_logger().info(f'[PUB] {msg.data}')

    # ─────────────────────────────────────────────
    # Test sequence
    # ─────────────────────────────────────────────
    def run_test(self):

        # 약간 대기 (bridge subscriber 연결 시간)
        time.sleep(1.0)

        # 1. 전진 3초
        self.get_logger().info('[TEST] forward 3 sec')
        self.publish_motor(120, 120)
        time.sleep(3.0)

        # 2. 후진 3초
        self.get_logger().info('[TEST] backward 3 sec')
        self.publish_motor(-120, -120)
        time.sleep(3.0)

        # 3. 정지
        self.get_logger().info('[TEST] emergency stop')
        self.publish_estop()

        time.sleep(0.5)

        self.get_logger().info('[DONE] test finished')


def main(args=None):
    rclpy.init(args=args)

    node = DrivingNode()

    try:
        # run_test 끝난 뒤 로그 잠깐 보기 위해 spin
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
