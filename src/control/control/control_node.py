#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
자율주행 제어 노드 (control_node)
- robot_perception 토픽(JSON 문자열)을 구독
- 단안 카메라 거리 추정으로 장애물까지 거리 계산
- FSM 으로 주행/정지/동작 결정
- motor 토픽으로 {"T":"m","L":v,"R":v} 또는 {"T":"e"} 발행

★ 콜백/발행 분리 구조 ★
- on_perception(콜백): FSM 상태 갱신 + "목표 명령(self.cmd)" 계산해서 저장만 한다
- publish_timer(50ms): 저장된 목표 명령을 끊김없이 계속 발행한다
  -> perception 이 느리거나 잠깐 끊겨도 ESP32 워치독(500ms)에 안 걸림

입력 데이터 구조:
  {
    "yolo": {
      "obstacles": [ {"type":"person","height":145.2}, {"type":"box","height":88.5} ],
      "traffic_light": "none"|"red"|"green_straight"|"green_left",
      "sign": "none"|"stop"|"speed_30"|"speed_50"
    },
    "opencv": { "lane_offset": 12, "is_stop_line": false, "is_crosswalk": true }
  }

입력 토픽:  /robot_perception  (std_msgs/String, JSON)
출력 토픽:  /motor_cmd         (std_msgs/String, JSON)
"""

import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


# =====================================================================
#  카메라 / 거리 추정 상수
# =====================================================================
# 거리 공식: Z = f * H / h   (h = obstacles[].height, 픽셀 높이)
FOCAL_PX     = 715.0   # 초점거리 [px]  (640x480 기준, f = Z*h/H 로 구함)
STOP_DIST_CM = 60.0    # 장애물이 이 거리 이내면 정지 [cm]

# 장애물 종류별 실제 높이 [cm] (거리 환산용)
# YOLO 가 주는 height(픽셀)와 짝지어 거리 계산. 실측값으로 보정 필요.
OBSTACLE_REAL_H = {
    "person": 14.0,   # 사람 실제 키 (cm) - 환경에 맞게 조정
    "box":     10.5,   # 박스 높이 (cm)
}
DEFAULT_REAL_H  = 10.5   # 미등록 타입의 기본 실제 높이 [cm]

# =====================================================================
#  주행 파라미터  (PWM: -255 ~ 255)
# =====================================================================
BASE_SPEED   = 90     # 기본 전진 PWM
SLOW_SPEED   = 50      # speed_30 등 감속 시 PWM
TURN_SPEED   = 70     # 좌/우 회전 동작 시 PWM
KP_STEER     = 0.6     # lane_offset -> 좌우 PWM 차이 변환 게인
MAX_PWM      = 230

# 정지선: is_stop_line 이 false 가 연속 몇 번이면 멈출지
STOPLINE_FALSE_LIMIT = 2

# 재전송 타이머 주기 [s]. ESP32 워치독(500ms)보다 충분히 짧아야 함.
PUBLISH_PERIOD = 0.05  # 50ms = 20Hz

# perception 이 이 시간 이상 끊기면 안전을 위해 정지 [s]
PERCEPTION_TIMEOUT = 0.5


def clamp(v, lo=-MAX_PWM, hi=MAX_PWM):
    return max(lo, min(hi, int(v)))


class DrivingNode(Node):

    def __init__(self):
        super().__init__('control_node')

        # ---- pub / sub ----
        self.pub = self.create_publisher(String, '/motor_cmd', 10)
        self.sub = self.create_subscription(
            String, '/robot_perception', self.on_perception, 10)

        # ---- 재전송 타이머 (콜백과 독립적으로 모터 명령 계속 발행) ----
        self.timer = self.create_timer(PUBLISH_PERIOD, self.publish_timer)

        # ---- FSM 상태 ----
        # DRIVING        : 차선추적 주행
        # STOP_FOR_OBJ   : 객체 60cm 도달 -> 정지 후 동작
        # STOP_AT_LINE   : 정지선에서 정지 (신호 대기)
        # EMERGENCY      : 비상정지
        self.state = 'DRIVING'

        # ---- 현재 목표 명령 (타이머가 이 값을 계속 발행) ----
        # kind: "m" (모터) 또는 "e" (비상정지)
        self.cmd = {"kind": "m", "L": 0, "R": 0}

        # ---- 정지선 false 연속 카운트 ----
        self.stopline_seen = False     # 한 번이라도 true 를 본 적 있는가
        self.false_count = 0

        # 동작 수행 1회성 플래그 (같은 객체에 반복 동작 방지)
        self.action_done = False

        # 마지막 perception 수신 시각 (타임아웃 감지용)
        self.last_perception_t = self.get_clock().now()

        # ---- 부팅 직후 1회 비상정지 (0.2s 뒤 한 번만 발행) ----
        self.boot_timer = self.create_timer(0.2, self.send_boot_estop)

        self.get_logger().info('Driving control node started.')

    # =================================================================
    #  목표 명령 설정 (콜백에서 호출 - 발행은 타이머가 담당)
    # =================================================================
    def set_motor(self, left, right):
        self.cmd = {"kind": "m", "L": clamp(left), "R": clamp(right)}

    def set_estop(self):
        self.cmd = {"kind": "e"}

    def set_stop(self):
        self.set_motor(0, 0)

    # =================================================================
    #  perception 필드 안전 추출
    #  yolo_detector 는 traffic_light/sign 을 dict 로 보낸다:
    #     "traffic_light": {"state": "red", "height": 12.3}
    #     "sign":          {"type":  "stop", "height": 8.1}
    #  과거 포맷(문자열)도 깨지지 않게 둘 다 처리한다.
    # =================================================================
    @staticmethod
    def _unwrap(value, default, key):
        if isinstance(value, dict):
            return value.get(key, default)
        if isinstance(value, str):
            return value
        return default

    # =================================================================
    #  부팅 직후 1회 비상정지 송신
    #  (discovery 가 자리잡도록 약간 지연 후 한 번만 쏘고 타이머 종료)
    # =================================================================
    def send_boot_estop(self):
        msg = String()
        msg.data = json.dumps({"T": "e"})
        self.pub.publish(msg)
        self.get_logger().info('[BOOT] initial E-STOP sent ({"T":"e"})')
        self.boot_timer.cancel()   # 1회만 실행

    # =================================================================
    #  재전송 타이머 콜백 (20Hz) - 실제로 토픽을 발행하는 유일한 곳
    # =================================================================
    def publish_timer(self):
        # perception 타임아웃 체크: 너무 오래 안 오면 안전 정지
        dt = (self.get_clock().now() - self.last_perception_t).nanoseconds * 1e-9
        if dt > PERCEPTION_TIMEOUT:
            # 이미 estop 이 아니면 정지로 덮어쓴다 (비상정지는 유지)
            if self.cmd.get("kind") != "e":
                self.set_stop()
            # 너무 잦은 로그 방지를 위해 상태만 바꿔둠
            self.state = 'STOP_AT_LINE'

        msg = String()
        if self.cmd.get("kind") == "e":
            msg.data = json.dumps({"T": "e"})
        else:
            msg.data = json.dumps(
                {"T": "m", "L": self.cmd["L"], "R": self.cmd["R"]})
        self.pub.publish(msg)

    # =================================================================
    #  거리 추정 (장애물 타입별 실제 높이 사용)
    # =================================================================
    def estimate_distance(self, real_h_cm, bbox_h_px):
        """단안 거리 추정: Z = f * H / h"""
        if bbox_h_px is None or bbox_h_px <= 0:
            return None
        return (FOCAL_PX * real_h_cm) / float(bbox_h_px)

    def nearest_obstacle_distance(self, obstacles):
        """obstacles 배열에서 가장 가까운(거리 최소) 장애물 거리를 반환.
        없으면 None. 각 장애물은 {"type":..., "height":픽셀높이}."""
        nearest = None
        for ob in obstacles:
            h_px = ob.get('height', None)
            if h_px is None or h_px <= 0:
                continue
            real_h = OBSTACLE_REAL_H.get(ob.get('type', ''), DEFAULT_REAL_H)
            d = self.estimate_distance(real_h, h_px)
            if d is None:
                continue
            if nearest is None or d < nearest:
                nearest = d
        return nearest

    # =================================================================
    #  차선 추적 조향 -> 목표 명령으로 저장
    # =================================================================
    def lane_follow(self, lane_offset, base_speed):
        """
        lane_offset: 차선 중앙 기준 픽셀 오차
            양수(+) -> 차선 중심이 오른쪽 -> 로봇이 왼쪽에 치우침 -> 우회전 필요
            음수(-) -> 좌회전 필요
        우회전: 오른쪽 바퀴를 줄이고 왼쪽을 키운다
        """
        diff = KP_STEER * lane_offset
        left  = base_speed + diff
        right = base_speed - diff
        self.set_motor(left, right)

    # =================================================================
    #  신호등 상태에 따른 동작 (거리 없이 상태만으로 판단)
    #  반환: True 면 "정지 유지"(콜백에서 즉시 return), False 면 통과
    # =================================================================
    def handle_traffic_light(self, tl, lane_offset):
        if tl == 'red':
            self.get_logger().info('[TL] red -> STOP & wait')
            self.set_stop()
            self.state = 'STOP_AT_LINE'
            return True   # 빨간불: 계속 정지 유지

        if tl == 'green_left':
            self.get_logger().info('[TL] green_left -> turn left')
            # 좌회전: 왼쪽 약하게/오른쪽 강하게 -> 좌선회
            self.set_motor(50, TURN_SPEED)
            self.state = 'DRIVING'
            return True   # 이번 사이클은 회전 명령으로 종료

        # green_straight / none -> 정지 유지 안 함 (정상 주행으로 진행)
        return False

    # =================================================================
    #  메인 콜백 - FSM 상태 갱신 + 목표 명령 계산 (발행은 안 함)
    # =================================================================
    def on_perception(self, msg):
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn('Bad JSON on /robot_perception')
            return

        # perception 수신 시각 갱신 (타임아웃 리셋)
        self.last_perception_t = self.get_clock().now()

        yolo   = data.get('yolo', {})
        opencv = data.get('opencv', {})

        obstacles    = yolo.get('obstacles', [])      # [{"type","height"}, ...]
        # yolo_detector 가 dict 로 보내므로 안에서 실제 값만 꺼낸다
        traffic_light= self._unwrap(yolo.get('traffic_light'), 'none', 'state')
        sign         = self._unwrap(yolo.get('sign'),          'none', 'type')
        lane_offset  = opencv.get('lane_offset', 0)
        is_stop_line = opencv.get('is_stop_line', False)
        is_crosswalk = opencv.get('is_crosswalk', False)  # 현재는 미사용(추후 확장)

        # -------------------------------------------------------------
        # 0순위: 장애물 거리 -> 60cm 이내면 정지 (충돌 방지)
        #        멀리 있으면 무시하고 주행 계속
        # -------------------------------------------------------------
        obs_dist = self.nearest_obstacle_distance(obstacles)
        if obs_dist is not None:
            self.get_logger().info(f'[OBS] nearest obstacle = {obs_dist:.1f} cm')
            if obs_dist <= STOP_DIST_CM:
                self.state = 'STOP_FOR_OBJ'
                self.set_stop()
                self.get_logger().warn(
                    f'[STOP_FOR_OBJ] obstacle within {STOP_DIST_CM:.0f}cm -> STOP')
                return

        # -------------------------------------------------------------
        # 1) 정지선 로직: true...true -> false 2연속이면 정지
        #    (카메라가 바닥 14cm 앞을 보므로 false 2번 후 멈추면
        #     정지선 바로 앞에서 멈춤)
        # -------------------------------------------------------------
        if is_stop_line:
            self.stopline_seen = True
            self.false_count = 0
        else:
            if self.stopline_seen:        # 한 번이라도 정지선을 본 뒤의 false 만 카운트
                self.false_count += 1

        if self.stopline_seen and self.false_count >= STOPLINE_FALSE_LIMIT:
            self.set_stop()
            self.get_logger().info(
                f'[STOP_AT_LINE] stopped at line (false x{self.false_count})')
            # 정지선 카운트 초기화 (다음 정지선을 위해)
            self.stopline_seen = False
            self.false_count = 0
            # 정지선에서 멈춘 뒤 신호등 판단:
            #   red  -> 계속 정지 / green_left -> 좌회전 / 그 외 -> 그대로 정지 유지
            self.handle_traffic_light(traffic_light, lane_offset)
            if traffic_light != 'green_straight':
                self.state = 'STOP_AT_LINE'
                return
            # green_straight 면 아래 주행 로직으로 진행

        # -------------------------------------------------------------
        # 2) 신호등 단독 판단 (정지선 없이도 빨간불/좌회전 신호 처리)
        # -------------------------------------------------------------
        if self.handle_traffic_light(traffic_light, lane_offset):
            return   # red(정지 유지) 또는 green_left(좌회전)면 여기서 종료

        # -------------------------------------------------------------
        # 3) is_crosswalk: 현재는 무시 (추후 감속/주의 로직 추가 위치)
        # -------------------------------------------------------------
        # if is_crosswalk:
        #     pass

        # -------------------------------------------------------------
        # 4) 기본: 차선 추적 주행
        # -------------------------------------------------------------
        self.state = 'DRIVING'
        self.action_done = False

        # 표지판 감속 반영 (yolo 는 'speed_limit' 을 보냄)
        speed = BASE_SPEED
        if sign in ('speed_limit', 'speed_30'):
            speed = SLOW_SPEED

        self.lane_follow(lane_offset, speed)


def main(args=None):
    rclpy.init(args=args)
    node = DrivingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 확실히 정지 명령 한 번 보냄
        try:
            stop = String()
            stop.data = json.dumps({"T": "m", "L": 0, "R": 0})
            node.pub.publish(stop)
        except Exception:
            pass
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
