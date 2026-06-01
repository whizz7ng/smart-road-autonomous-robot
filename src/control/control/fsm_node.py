#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
모터 테스트 노드
- /motor_cmd 로 {"T":"m","L":90,"R":90} 를 20Hz로 10초간 계속 발행
- 10초 경과 후 정지({"T":"m","L":0,"R":0}) 발행 후 종료
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

PWM            = 80      # 전진 PWM
RUN_TIME       = 25.0    # 발행 지속 시간 [s]
PUBLISH_PERIOD = 0.05    # 50ms = 20Hz (ESP32 워치독 500ms 대비 충분히 짧음)


class MotorTestNode(Node):

    def __init__(self):
        super().__init__('fsm_node')

        self.pub = self.create_publisher(String, '/fsm_cmd', 10)
        self.start_t = self.get_clock().now()
        self.timer = self.create_timer(PUBLISH_PERIOD, self.publish_timer)

        self.get_logger().info(f'Motor test started: L={PWM}, R={PWM} for {RUN_TIME}s')

    def publish_timer(self):
        elapsed = (self.get_clock().now() - self.start_t).nanoseconds * 1e-9

        msg = String()
        if elapsed < RUN_TIME:
            msg.data = json.dumps({"T": "m", "L": PWM, "R": PWM})
        else:
            # 10초 경과 -> 정지 후 종료
            msg.data = json.dumps({"T": "m", "L": 0, "R": 0})
            self.pub.publish(msg)
            self.get_logger().info('Done. Stopping motors and shutting down.')
            self.timer.cancel()
            rclpy.shutdown()
            return

        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MotorTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 확실히 정지 한 번 더
        try:
            stop = String()
            stop.data = json.dumps({"T": "m", "L": 0, "R": 0})
            node.pub.publish(stop)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
