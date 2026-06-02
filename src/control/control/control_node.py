#!/usr/bin/env python3
"""
control_node: /fsm_cmd (String) -> /cmd_vel (geometry_msgs/Twist)

  PID / 바퀴 동기화는 ESP32 펌웨어가 담당.
  이 노드는 고수준 명령을 속도 목표로 변환만 한다.

    "forward" -> linear.x = forward_speed, angular.z = 0.0
    "stop"    -> 0, 0
    (그 외 알 수 없는 명령은 안전상 정지)

  cmd_vel 은 publish_rate(20Hz) 로 계속 발행한다.
  -> bridge_node(10Hz 재송신, 0.5s stale 정지) / ESP32 워치독(500ms) 유지.

  bridge_node 가 cmd_vel -> 좌/우 RPM 역기구학을 처리하므로
  여기서 바퀴 RPM 을 직접 다루지 않는다.

속도 환산 참고 (bridge twist_to_wheel_rpm 기준):
  rpm = v * 60 / (2*pi*0.0325) ≈ v * 293.8
  forward_speed=0.12 m/s -> 약 35 RPM (ESP32 튜닝 검증 구간)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Twist


CMD_FORWARD = "forward"
CMD_STOP = "stop"


class ControlNode(Node):
    def __init__(self):
        super().__init__("control_node")

        # ----- 파라미터 -----
        self.declare_parameter("forward_speed", 0.12)   # m/s (~35 RPM)
        self.declare_parameter("publish_rate", 20.0)    # Hz
        self.forward_speed = float(self.get_parameter("forward_speed").value)
        rate = float(self.get_parameter("publish_rate").value)

        # 시작은 안전상 정지
        self.current_cmd = CMD_STOP

        # ----- pub/sub -----
        self.create_subscription(String, "fsm_cmd", self.cmd_cb, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "cmd_vel", 10)
        self.create_timer(1.0 / rate, self.publish_cmd_vel)

        self.get_logger().info(
            f"control_node up. forward_speed={self.forward_speed:.3f} m/s")

    def cmd_cb(self, msg: String):
        self.current_cmd = msg.data.strip().lower()

    def publish_cmd_vel(self):
        tw = Twist()
        if self.current_cmd == CMD_FORWARD:
            tw.linear.x = self.forward_speed
            tw.angular.z = 0.0
        else:  # stop / unknown -> 안전 정지
            tw.linear.x = 0.0
            tw.angular.z = 0.0
        self.cmd_vel_pub.publish(tw)


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
