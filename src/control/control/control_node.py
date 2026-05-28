#!/usr/bin/env python3
"""
Test Driving Node
- /motor_cmd 로 실험용 PWM 명령 전송
- 동작:
    1) 전진 3초  (L=120,  R=120)
    2) 후진 3초  (L=-120, R=-120)
    3) 비상정지 1회 전송 후 종료
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


MOTOR_CMD_TOPIC = '/motor_cmd'
PUBLISH_RATE_HZ = 20          # 초당 20번 (= 50ms 주기)로 명령 재전송
PUBLISH_PERIOD = 1.0 / PUBLISH_RATE_HZ


class DrivingNode(Node):
    def __init__(self):
        super().__init__('control_node')

        self.pub = self.create_publisher(String, MOTOR_CMD_TOPIC, 10)

        self.get_logger().info('[READY] control_node started')

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

    def publish_estop(self):
        payload = {"T": "e"}

        msg = String()
        msg.data = json.dumps(payload)

        self.pub.publish(msg)

        self.get_logger().info(f'[PUB] {msg.data}')

    # ─────────────────────────────────────────────
    # 지정된 시간 동안 같은 명령을 일정 주기로 계속 publish
    # ─────────────────────────────────────────────
    def drive_for(self, left_pwm: int, right_pwm: int, duration_sec: float):
        self.get_logger().info(
            f'[DRIVE] L={left_pwm}, R={right_pwm} for {duration_sec}s'
        )

        start = time.time()
        last_log = 0.0

        while time.time() - start < duration_sec:
            self.publish_motor(left_pwm, right_pwm)

            # 로그는 1초마다 한 번만 (스팸 방지)
            elapsed = time.time() - start
            if elapsed - last_log >= 1.0:
                self.get_logger().info(
                    f'  ... t={elapsed:.1f}s  L={left_pwm}, R={right_pwm}'
                )
                last_log = elapsed

            # spin_once로 ROS 콜백도 처리하면서 sleep
            rclpy.spin_once(self, timeout_sec=PUBLISH_PERIOD)

    # ─────────────────────────────────────────────
    # Test sequence
    # ─────────────────────────────────────────────
    def run_test(self):
        # 약간 대기 (bridge subscriber 연결 시간)
        time.sleep(1.0)

        # 1. 전진 3초
        self.get_logger().info('[TEST] forward 15 sec')
        self.drive_for(120, 120, 15.0)

        # 2. 정지 (estop은 latch 되므로 한 번이면 충분하지만, 안전하게 몇 번 보냄)
        self.get_logger().info('[TEST] emergency stop')
        for _ in range(5):
            self.publish_estop()
            time.sleep(0.05)

        self.get_logger().info('[DONE] test finished')


def main(args=None):
    rclpy.init(args=args)

    node = DrivingNode()

    try:
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
