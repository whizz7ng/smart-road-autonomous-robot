#!/usr/bin/env python3
"""
fsm_node: /robot_perception (JSON) -> 주행 상태 결정 -> /fsm_cmd (String)

상태:
  DRIVING : 정상주행. 현재는 단순 "forward" 발행.
            (추후 perception 종합 판단으로 forward/slow/lane_change 등 분기 예정)
  STOPPED : 정지. 진입 시각 기록 후 STOP_HOLD_SEC 경과하면 DRIVING 복귀.

정지선 디바운스:
  is_stop_line == True  누적 TRUE_THRESH 회   -> armed (정지선 확인)
  armed 이후 False 가 FALSE_THRESH 회 연속    -> STOPPED 진입
    (정지선이 카메라 하단 밖으로 빠진 시점 = 차체가 선에 도달)

재출발:
  STOPPED 진입 후 STOP_HOLD_SEC(=1.0s) 경과 -> DRIVING 복귀
  복귀 시 정지선 관련 카운터 전부 리셋 + COOLDOWN_SEC(=2.0s) 동안 정지선 입력 무시
  (같은 선 잔상으로 재트리거되는 것 방지)
"""

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# 주행 상태
STATE_DRIVING = "DRIVING"
STATE_STOPPED = "STOPPED"

# /fsm_cmd 명령값
CMD_FORWARD = "forward"
CMD_STOP = "stop"


class FsmNode(Node):
    def __init__(self):
        super().__init__("fsm_node")

        # ----- 튜닝 파라미터 -----
        self.declare_parameter("true_thresh", 5)     # armed 까지 필요한 true 누적 횟수
        self.declare_parameter("false_thresh", 2)    # armed 이후 정지 트리거 false 연속 횟수
        self.declare_parameter("stop_hold_sec", 1.0) # STOPPED 유지 시간
        self.declare_parameter("cooldown_sec", 2.0)  # 재출발 후 정지선 무시 시간
        self.declare_parameter("publish_rate", 20.0)

        self.TRUE_THRESH   = int(self.get_parameter("true_thresh").value)
        self.FALSE_THRESH  = int(self.get_parameter("false_thresh").value)
        self.STOP_HOLD_SEC = float(self.get_parameter("stop_hold_sec").value)
        self.COOLDOWN_SEC  = float(self.get_parameter("cooldown_sec").value)
        rate               = float(self.get_parameter("publish_rate").value)

        # ----- FSM 상태 -----
        self.state = STATE_DRIVING
        self.stop_enter_time = 0.0    # STOPPED 진입 시각
        self.cooldown_until = 0.0     # 이 시각까지 정지선 입력 무시

        # 정지선 디바운스 상태
        self.true_count = 0
        self.false_count = 0
        self.armed = False

        # 최신 perception (다음 단계에서 장애물/신호등 판단할 때 사용)
        self.last_perception = None

        # ----- pub/sub -----
        self.create_subscription(
            String, "robot_perception", self.perception_cb, 10)
        self.cmd_pub = self.create_publisher(String, "fsm_cmd", 10)

        # 명령은 타이머로 주기 발행 + STOPPED 만료 체크도 여기서
        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"fsm_node up. true_thresh={self.TRUE_THRESH} "
            f"false_thresh={self.FALSE_THRESH} "
            f"stop_hold={self.STOP_HOLD_SEC}s cooldown={self.COOLDOWN_SEC}s")

    # ===== perception 콜백: 상태 갱신 =====
    def perception_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn("perception JSON parse 실패")
            return

        self.last_perception = data   # 향후 종합 판단용

        cv = data.get("opencv", {}) or {}
        is_stop_line = bool(cv.get("is_stop_line", False))
        self.update_stop_line(is_stop_line)

    def update_stop_line(self, is_stop_line: bool):
        # STOPPED 상태에서는 정지선 입력 무시 (타이머가 해제)
        if self.state == STATE_STOPPED:
            return

        # 재출발 직후 쿨다운: 같은 선 잔상 재트리거 방지
        if time.monotonic() < self.cooldown_until:
            return

        if not self.armed:
            # --- 접근 단계: true 누적 ---
            if is_stop_line:
                self.true_count += 1
                if self.true_count >= self.TRUE_THRESH:
                    self.armed = True
                    self.false_count = 0
                    self.get_logger().info(
                        f"stop line CONFIRMED (true x{self.true_count}) -> armed")
            # 접근 중 false 는 무시 (누적값 유지)
        else:
            # --- armed 단계: 선이 시야에서 빠지는지 감시 ---
            if is_stop_line:
                self.false_count = 0
            else:
                self.false_count += 1
                if self.false_count >= self.FALSE_THRESH:
                    self.enter_stopped("stop_line")

    # ===== 상태 전이 =====
    def enter_stopped(self, reason: str):
        self.state = STATE_STOPPED
        self.stop_enter_time = time.monotonic()
        self.get_logger().info(f"-> STOPPED (reason={reason})")

    def enter_driving(self):
        self.state = STATE_DRIVING
        # 정지선 카운터 리셋 + 쿨다운 시작
        self.armed = False
        self.true_count = 0
        self.false_count = 0
        self.cooldown_until = time.monotonic() + self.COOLDOWN_SEC
        self.get_logger().info(
            f"-> DRIVING (cooldown {self.COOLDOWN_SEC:.1f}s)")

    # ===== 주기 tick: 타이머 체크 + 명령 발행 =====
    def tick(self):
        # STOPPED 만료 체크
        if self.state == STATE_STOPPED:
            if time.monotonic() - self.stop_enter_time >= self.STOP_HOLD_SEC:
                self.enter_driving()

        # 명령 발행
        msg = String()
        msg.data = CMD_STOP if self.state == STATE_STOPPED else CMD_FORWARD
        self.cmd_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FsmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
