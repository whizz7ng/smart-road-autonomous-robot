
#!/usr/bin/env python3
"""
fsm_node: /robot_perception (JSON) -> 주행 상태 결정 + 차선 조향 -> /fsm_cmd (JSON String)

상태:
  DRIVING     : 정상주행. "forward" + lane 조향(angular_z) 발행.
  STOPPED     : 정지. 진입 시각 기록 후 STOP_HOLD_SEC 경과하면 DRIVING 복귀.
  LANE_CHANGE : 오픈 루프 3단계 시퀀스
                  ROTATE_OUT (제자리 회전, LC_ROT_SEC)
                  STRAIGHT   (직진 LC_STRAIGHT_DIST_M, 시간=dist/speed)
                  ROTATE_IN  (반대 방향 제자리 회전, LC_ROT_SEC)
                완료 후 DRIVING + target_lane 클리어.

차선 변경 트리거:
  ros2 topic pub --once /target_lane std_msgs/msg/Int32 "data: 1"
  ros2 topic pub --once /target_lane std_msgs/msg/Int32 "data: 2"

lane_number 신뢰 규약:
  lane_state == "both" 일 때만 current_lane 갱신.
  그 외(left/right)는 노이즈 가능성이 있어 무시. -> 잘못된 LC 재트리거 방지.

부호 규약:
  2 -> 1 (왼쪽 차선): ROTATE_OUT az = +LC_ROT_AZ,  ROTATE_IN az = -LC_ROT_AZ
  1 -> 2 (오른쪽 차선): ROTATE_OUT az = -LC_ROT_AZ, ROTATE_IN az = +LC_ROT_AZ
  실주행에서 방향 반대면 lane_change_rotate_az 부호만 뒤집어라.
"""

import json
import time
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
from collections import deque, Counter


# 주행 상태
STATE_DRIVING     = "DRIVING"
STATE_STOPPED     = "STOPPED"
STATE_LANE_CHANGE = "LANE_CHANGE"

# 차선 변경 sub-phase
LC_PHASE_PRE_STOP   = "PRE_STOP"     # 시작 전
LC_PHASE_ROTATE_OUT = "ROTATE_OUT"
LC_PHASE_STOP1      = "STOP1"        # 회전 후
LC_PHASE_STRAIGHT   = "STRAIGHT"
LC_PHASE_STOP2      = "STOP2"        # 직진 후
LC_PHASE_ROTATE_IN  = "ROTATE_IN"
LC_PHASE_STOP3      = "STOP3"        # 반대회전 후(복귀 전 안정)

LT_PHASE_PRE_STOP = "LT_PRE_STOP"
LT_PHASE_STRAIGHT = "LT_STRAIGHT"
LT_PHASE_CURVE    = "LT_CURVE"

# /fsm_cmd 명령값
CMD_FORWARD = "forward"
CMD_STOP = "stop"

STATE_LEFT_TURN = "LEFT_TURN"

LT_PHASE_STRAIGHT = "LT_STRAIGHT"
LT_PHASE_CURVE    = "LT_CURVE"

# ===== 객체 거리 추정 (픽셀비례) =====
REF_DIST_CM = 70.0
OBJECT_REF_PX = {
    "traffic_light": 63.0,
    "sign":          29.0,
    "construction":  55.0,
    "traffic_light_left": 46.0,
}


def resolve_class_key(category: str, item: dict) -> str:
    if category == "traffic_lights":
        state = str(item.get("state", ""))
        if state.endswith("_left"):
            return "traffic_light_left"
        return "traffic_light"
    if category == "signs":
        return "sign"
    if category == "obstacles":
        return item.get("type", "unknown")
    return "unknown"


def estimate_distance_cm(class_key: str, h_px):
    ref = OBJECT_REF_PX.get(class_key)
    if ref is None:
        return None
    try:
        h = float(h_px)
    except (TypeError, ValueError):
        return None
    if h <= 0:
        return None
    return REF_DIST_CM * ref / h


class FsmNode(Node):
    def __init__(self):
        super().__init__("fsm_node")

        # ----- 튜닝 파라미터 -----
        #self.declare_parameter("true_thresh", 6)
        self.declare_parameter("false_thresh", 3)
        self.declare_parameter("stop_hold_sec", 2.5)
        self.declare_parameter("cooldown_sec", 2.0)
        self.declare_parameter("publish_rate", 20.0)
        # 차선 조향
        self.declare_parameter("frame_width", 640)       # px
        self.declare_parameter("view_width_cm", 51.0)    # 화면 가로 실제 길이
        self.declare_parameter("steer_kp", 0.02)        # (rad/s) / cm
        self.declare_parameter("steer_max", 0.6)         # rad/s clamp
        self.declare_parameter("steer_deadband_cm", 0)
        # 차선 변경 (오픈 루프)
        self.declare_parameter("lane_change_rotate_az",  1.6)   # rad/s, 회전 각속도 크기 (>0)
        self.declare_parameter("lane_change_rotate_sec", 0.7)   # s, 한 번 회전 지속
        self.declare_parameter("lane_change_straight_dist_m", 0.34)  # m
        self.declare_parameter("lane_change_straight_speed",   0.23) # m/s, control_node forward_speed 와 맞춰라
        self.declare_parameter("lane_number_window", 20)     # 최근 N개 샘플
        self.declare_parameter("lane_number_min_samples", 14) # 결정에 필요한 최소 표 수
        self.declare_parameter("lane_change_mid_stop_sec", 0.6)
        
        self.declare_parameter("avoid_class", "construction")
        self.declare_parameter("avoid_dist_cm", 65.0)
        self.declare_parameter("avoid_cooldown_sec", 8.0)

        # 표지판 트리거
        self.declare_parameter("slow_class", "speed_limit")
        self.declare_parameter("slow_trigger_dist_cm", 55.0)
        self.declare_parameter("forward_speed", 0.23)     # control_node 와 맞춰라
        self.declare_parameter("slow_speed", 0.18)
        self.declare_parameter("slow_duration_sec", 5.0)
        self.declare_parameter("stop_sign_class", "stop")
        self.declare_parameter("stop_sign_dist_cm", 55.0)
        self.declare_parameter("stop_sign_cooldown_sec", 6.0)
        
        self.declare_parameter("traffic_light_dist_cm", 70.0)   # 이 거리 이내 신호등만 반응
        self.declare_parameter("green_blink_grace_sec", 2.5)    # green 본 뒤 off를 직진으로 보는 시간
        
        self.declare_parameter("left_turn_dist_cm", 270.0)
                # 좌회전 신호 트리거 (px 직접)
        self.declare_parameter("left_change_min_px", 15.0)  # 차선변경 트리거 (~270cm)
        self.declare_parameter("left_turn_min_px",   23.0)  # 좌회전 실행 트리거 (~140cm)
        self.declare_parameter("left_stop_min_px", 23.0)   # 이 크기 이상이면 정지 판단 (~140cm)
        # 좌회전 실행 시퀀스 (하드코딩)
        self.declare_parameter("left_turn_straight_dist_m", 0.50)
        self.declare_parameter("left_turn_straight_speed",  0.23)  # = forward_speed
        self.declare_parameter("left_turn_curve_dist_m",    1.00)
        self.declare_parameter("left_turn_curve_deg",       -90.0)
        self.declare_parameter("left_turn_curve_speed",     0.15)  # 곡선 선속도(천천히)
        self.declare_parameter("left_turn_cooldown_sec",    8.0)
        self.declare_parameter("pre_maneuver_stop_sec", 0.3)
        self.declare_parameter("left_turn_window", 7)    # 최근 N프레임
        self.declare_parameter("left_turn_confirm", 4)   # 그중 임계 이상이 M번

        
        #self.TRUE_THRESH   = int(self.get_parameter("true_thresh").value)
        self.FALSE_THRESH  = int(self.get_parameter("false_thresh").value)
        self.STOP_HOLD_SEC = float(self.get_parameter("stop_hold_sec").value)
        self.COOLDOWN_SEC  = float(self.get_parameter("cooldown_sec").value)
        rate               = float(self.get_parameter("publish_rate").value)

        frame_width        = float(self.get_parameter("frame_width").value)
        view_width_cm      = float(self.get_parameter("view_width_cm").value)
        self.PX_TO_CM      = view_width_cm / frame_width
        self.STEER_KP      = float(self.get_parameter("steer_kp").value)
        self.STEER_MAX     = float(self.get_parameter("steer_max").value)
        self.STEER_DEADBAND_CM = float(self.get_parameter("steer_deadband_cm").value)

        self.LC_ROT_AZ        = abs(float(self.get_parameter("lane_change_rotate_az").value))
        self.LC_ROT_SEC       = float(self.get_parameter("lane_change_rotate_sec").value)
        self.LC_STRAIGHT_DIST = float(self.get_parameter("lane_change_straight_dist_m").value)
        self.LC_STRAIGHT_SPD  = float(self.get_parameter("lane_change_straight_speed").value)
        self.LC_STRAIGHT_SEC  = (self.LC_STRAIGHT_DIST / self.LC_STRAIGHT_SPD
                                 if self.LC_STRAIGHT_SPD > 0 else 0.0)
        self.LC_MID_STOP_SEC = float(self.get_parameter("lane_change_mid_stop_sec").value)
        
        self.PRE_STOP_SEC = float(self.get_parameter("pre_maneuver_stop_sec").value)
        
        self.AVOID_CLASS        = str(self.get_parameter("avoid_class").value)
        self.AVOID_DIST_CM      = float(self.get_parameter("avoid_dist_cm").value)
        self.AVOID_COOLDOWN_SEC = float(self.get_parameter("avoid_cooldown_sec").value)
        
        self.SLOW_CLASS         = str(self.get_parameter("slow_class").value)
        self.SLOW_TRIG_DIST     = float(self.get_parameter("slow_trigger_dist_cm").value)
        fwd_speed               = float(self.get_parameter("forward_speed").value)
        slow_speed              = float(self.get_parameter("slow_speed").value)
        self.SLOW_SCALE         = max(0.0, min(1.0, slow_speed / fwd_speed)) if fwd_speed > 0 else 1.0
        self.SLOW_DURATION_SEC  = float(self.get_parameter("slow_duration_sec").value)
        self.STOP_SIGN_CLASS    = str(self.get_parameter("stop_sign_class").value)
        self.STOP_SIGN_DIST     = float(self.get_parameter("stop_sign_dist_cm").value)
        self.STOP_SIGN_COOLDOWN = float(self.get_parameter("stop_sign_cooldown_sec").value)
        
        self.TL_DIST_CM        = float(self.get_parameter("traffic_light_dist_cm").value)
        self.GREEN_GRACE_SEC   = float(self.get_parameter("green_blink_grace_sec").value)
        self.LEFT_TURN_DIST_CM = float(self.get_parameter("left_turn_dist_cm").value)
        self.LEFT_STOP_PX = float(self.get_parameter("left_stop_min_px").value)
        
        self.LC_TRIGGER_PX = float(self.get_parameter("left_change_min_px").value)
        self.LT_TRIGGER_PX = float(self.get_parameter("left_turn_min_px").value)

        lt_fwd = float(self.get_parameter("left_turn_straight_speed").value)
        self.LT_STRAIGHT_DIST = float(self.get_parameter("left_turn_straight_dist_m").value)
        self.LT_STRAIGHT_SEC  = self.LT_STRAIGHT_DIST / lt_fwd if lt_fwd > 0 else 0.0

        self.LT_CURVE_DIST = float(self.get_parameter("left_turn_curve_dist_m").value)
        self.LT_CURVE_DEG  = float(self.get_parameter("left_turn_curve_deg").value)
        lt_cv_spd          = float(self.get_parameter("left_turn_curve_speed").value)
        self.LT_CURVE_SEC   = self.LT_CURVE_DIST / lt_cv_spd if lt_cv_spd > 0 else 0.0
        w_mag = (math.radians(self.LT_CURVE_DEG) / self.LT_CURVE_SEC
                 if self.LT_CURVE_SEC > 0 else 0.0)
        self.LT_CURVE_AZ    = -w_mag                    # 왼쪽=음수(차선변경과 동일 규약). 반대면 +w_mag
        self.LT_CURVE_SCALE = lt_cv_spd / lt_fwd if lt_fwd > 0 else 0.0
        self.LEFT_TURN_COOLDOWN = float(self.get_parameter("left_turn_cooldown_sec").value)
        
        self.LT_TRIGGER_WINDOW  = int(self.get_parameter("left_turn_window").value)
        self.LT_TRIGGER_CONFIRM = int(self.get_parameter("left_turn_confirm").value)
        
        # ----- FSM 상태 -----
        self.state = STATE_DRIVING
        self.stop_enter_time = 0.0
        self.cooldown_until = 0.0

        # 정지선 디바운스 상태
        self.true_count = 0
        self.false_count = 0
        self.armed = False
        
        
        self.LN_WINDOW      = int(self.get_parameter("lane_number_window").value)
        self.LN_MIN_SAMPLES = int(self.get_parameter("lane_number_min_samples").value)
        self.lane_history = deque(maxlen=self.LN_WINDOW)
        self.lt_height_window = deque(maxlen=self.LT_TRIGGER_WINDOW)
        self.lc_height_window = deque(maxlen=self.LT_TRIGGER_WINDOW)
        

        # 차선 변경 상태
        self.target_lane = None        # 외부에서 받은 목표 차선 (1 or 2). 완료 시 None.
        self.current_lane = None       # both 일 때만 갱신되는 신뢰값
        self.lc_phase = None
        self.lc_phase_start = 0.0
        self.lc_az_out = 0.0           # ROTATE_OUT 시 angular_z (부호 포함)
        self.lc_az_in  = 0.0           # ROTATE_IN  시 angular_z (부호 포함)
        
        self.last_avoid_time = 0.0
        self.slow_until = 0.0
        self.last_stop_sign_time = 0.0
        self.last_green_time = 0.0
        
        self.lt_phase = None
        self.lt_phase_start = 0.0
        self.last_left_turn_time = 0.0
        # 최신 perception 산출물 (fsm_cmd 에 실어 발행)
        self.latest_objects = {"obstacles": [], "traffic_lights": [], "signs": []}
        self.latest_lane = {"offset_px": None, "offset_cm": None,
                            "angular_z": 0.0, "state": None,
                            "number_raw": None, "number_trusted": None}

        # ----- pub/sub -----
        self.create_subscription(
            String, "robot_perception", self.perception_cb, 10)
        self.create_subscription(
            Int32, "target_lane", self.target_lane_cb, 10)
        self.cmd_pub = self.create_publisher(String, "fsm_cmd", 10)

        self.create_timer(1.0 / rate, self.tick)

        self.get_logger().info(
            f"fsm_node up. steer_kp={self.STEER_KP} "
            f"LC: rot_az={self.LC_ROT_AZ:+.2f} rot_sec={self.LC_ROT_SEC:.2f} "
            f"straight_dist={self.LC_STRAIGHT_DIST:.2f}m "
            f"@ {self.LC_STRAIGHT_SPD:.2f}m/s -> {self.LC_STRAIGHT_SEC:.2f}s")

    # ===== /target_lane 콜백 =====
    def target_lane_cb(self, msg: Int32):
        n = int(msg.data)
        if n not in (1, 2):
            self.get_logger().warn(f"target_lane={n} 무시 (1 또는 2만 허용)")
            return
        if self.current_lane is not None and self.current_lane == n:
            self.get_logger().info(
                f"target_lane={n} 이미 같은 차선 (current={self.current_lane}) -> 무시")
            return
        if self.state == STATE_LANE_CHANGE:
            self.get_logger().warn(
                f"target_lane={n} 무시 (이미 LANE_CHANGE 진행중)")
            return
        self.target_lane = n
        self.get_logger().info(
            f"target_lane <- {n} (current={self.current_lane}, state={self.state})")

    # ===== perception 콜백 =====
    def perception_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
        except (json.JSONDecodeError, TypeError):
            self.get_logger().warn("perception JSON parse 실패")
            return

        yolo = data.get("yolo", {}) or {}
        self.latest_objects = self.enrich_with_distance(yolo)

        cv = data.get("opencv", {}) or {}
        #is_stop_line = bool(cv.get("is_stop_line", False))
        #self.update_stop_line(is_stop_line)
        ln_raw = cv.get("lane_number")
        try:
            ln_i = int(ln_raw) if ln_raw is not None else None
        except (TypeError, ValueError):
            ln_i = None

        # DRIVING 중에만 표 수집 (회전/직진 기동 중 노이즈 차단)
        if self.state == STATE_DRIVING and ln_i in (1, 2):
            self.lane_history.append(ln_i)
        if len(self.lane_history) >= self.LN_MIN_SAMPLES:
            self.current_lane = Counter(self.lane_history).most_common(1)[0][0]
        # samples 부족(부팅 직후)이면 current_lane 은 기존값 유지 — None으로 덮지 않음

        # 차선 조향 갱신
        self.update_lane(cv.get("lane_offset"))
        self.maybe_trigger_avoid()
        self.maybe_trigger_left_lane_change()
        self.maybe_trigger_left_turn()
        self.maybe_trigger_slow()
        self.maybe_trigger_stop_sign()

    def enrich_with_distance(self, yolo: dict) -> dict:
        out = {}
        for category in ("obstacles", "traffic_lights", "signs"):
            enriched = []
            for it in (yolo.get(category, []) or []):
                key = resolve_class_key(category, it)
                dist = estimate_distance_cm(key, it.get("height"))
                new_it = dict(it)
                new_it["dist_cm"] = round(dist, 1) if dist is not None else None
                enriched.append(new_it)
            out[category] = enriched
        return out

    # ===== 차선 조향 =====
    def update_lane(self, lane_offset_px):
        off_cm, az = self.compute_steer(lane_offset_px)
        self.latest_lane = {
            "offset_px": lane_offset_px,
            "offset_cm": round(off_cm, 1) if off_cm is not None else None,
            "angular_z": round(az, 4),
        }

    def compute_steer(self, lane_offset_px):
        if lane_offset_px is None:
            return None, 0.0
        try:
            off_px = float(lane_offset_px)
        except (TypeError, ValueError):
            return None, 0.0
        off_cm = off_px * self.PX_TO_CM
        err = off_cm
        if abs(err) < self.STEER_DEADBAND_CM:
            err = 0.0
        az = self.STEER_KP * err
        az = max(-self.STEER_MAX, min(self.STEER_MAX, az))
        return off_cm, az

    # ===== 정지선 디바운스 =====
    '''
    def update_stop_line(self, is_stop_line: bool):
        if self.state != STATE_DRIVING:   # STOPPED / LANE_CHANGE 중엔 무시
            return
        if time.monotonic() < self.cooldown_until:
            return

        if not self.armed:
            if is_stop_line:
                self.true_count += 1
                if self.true_count >= self.TRUE_THRESH:
                    self.armed = True
                    self.false_count = 0
                    self.get_logger().info(
                        f"stop line CONFIRMED (true x{self.true_count}) -> armed")
        else:
            if is_stop_line:
                self.false_count = 0
            else:
                self.false_count += 1
                if self.false_count >= self.FALSE_THRESH:
                    self.enter_stopped("stop_line")
    '''
    # ===== 상태 전이 =====
    def enter_stopped(self, reason: str):
        self.state = STATE_STOPPED
        self.stop_enter_time = time.monotonic()
        self.get_logger().info(f"-> STOPPED (reason={reason})")

    def enter_driving(self, reason="", assume_lane=None):
        self.state = STATE_DRIVING
        self.armed = False
        self.true_count = 0
        self.false_count = 0
        self.cooldown_until = time.monotonic() + self.COOLDOWN_SEC
        self.lc_phase = None
        self.lt_phase = None        # 3번: 좌회전 완료 후 stale phase 제거
        if assume_lane in (1, 2):
            # 차선 변경 완료 -> 새 차선을 확정값으로 시딩 (복귀 직후 None 구간 제거)
            self.lane_history.clear()
            for _ in range(self.LN_WINDOW):
                self.lane_history.append(assume_lane)
            self.current_lane = assume_lane
        # 정지선/정지표지/좌회전 복귀는 lane_history 보존 -> 같은 차선 정보 유지
        self.get_logger().info(
            f"-> DRIVING (cooldown {self.COOLDOWN_SEC:.1f}s){' '+reason if reason else ''}")
        
    def maybe_trigger_avoid(self):
        """construction 이 AVOID_DIST_CM 이내면 반대 차선으로 target_lane 설정."""
        if self.state != STATE_DRIVING:
            return
        if self.current_lane is None:
            return
        if self.target_lane is not None:
            return  # 이미 트리거됨
        if time.monotonic() - self.last_avoid_time < self.AVOID_COOLDOWN_SEC:
            return
        for it in self.latest_objects.get("obstacles", []):
            if it.get("type") != self.AVOID_CLASS:
                continue
            dist = it.get("dist_cm")
            if dist is None or dist > self.AVOID_DIST_CM:
                continue
            opposite = 1 if self.current_lane == 2 else 2
            self.target_lane = opposite
            self.last_avoid_time = time.monotonic()
            self.get_logger().info(
                f"AVOID: {self.AVOID_CLASS} @ {dist:.1f}cm "
                f"(current_lane={self.current_lane}) -> target_lane={opposite}")
            break

    def traffic_light_blocks(self) -> bool:
            """일반 신호등(red/green/off) 기준 정지 판단. 좌회전(_left)은 제외."""
            lights = [l for l in self.latest_objects.get("traffic_lights", [])
                    if l.get("dist_cm") is not None and l["dist_cm"] <= self.TL_DIST_CM
                    and not str(l.get("state", "")).endswith("_left")
                    and not (self.current_lane == 1 and str(l.get("state", "")) == "off")]
            if not lights:
                return False

            tl = min(lights, key=lambda l: l["dist_cm"])
            state = tl.get("state")
            now = time.monotonic()

            if state == "green_straight":
                self.last_green_time = now
                return False
            if state == "red":
                self.last_green_time = 0.0
                return True
            if now - self.last_green_time <= self.GREEN_GRACE_SEC:
                return False
            return True
    
    def left_traffic_light_blocks(self) -> bool:
        """좌회전 신호 red_left/off_left 가 가까우면(>=LEFT_STOP_PX) 정지.
           단, 일반 신호등이 같이 보이면 일반 신호 우선이라 무시."""
        if self._has_normal_traffic_light():
            return False
        for tl in self.latest_objects.get("traffic_lights", []):
            state = str(tl.get("state", ""))
            if state not in ("red_left", "off_left"):
                continue
            try:
                h = float(tl.get("height"))
            except (TypeError, ValueError):
                continue
            if h >= self.LEFT_STOP_PX:
                return True
        return False
    
    def _has_normal_traffic_light(self) -> bool:
        """프레임에 일반(비좌회전) 신호등이 하나라도 있으면 True.
        단, 1차선에선 off(=off_left 와 겹쳐 잡히는 노이즈)는 무시."""
        for tl in self.latest_objects.get("traffic_lights", []):
            state = str(tl.get("state", ""))
            if not state or state.endswith("_left"):
                continue
            if self.current_lane == 1 and state == "off":
                continue
            return True
        return False

    def _find_sign(self, class_key: str, max_dist_cm: float):
        for it in self.latest_objects.get("signs", []):
            if it.get("type") != class_key:
                continue
            dist = it.get("dist_cm")
            if dist is None or dist > max_dist_cm:
                continue
            return it
        return None

    def maybe_trigger_slow(self):
        if self.state != STATE_DRIVING:
            return
        if self._find_sign(self.SLOW_CLASS, self.SLOW_TRIG_DIST) is None:
            return
        now = time.monotonic()
        was_slow = now < self.slow_until
        self.slow_until = now + self.SLOW_DURATION_SEC  # 계속 보이면 갱신
        if not was_slow:
            self.get_logger().info(
                f"SLOW: {self.SLOW_CLASS} -> scale={self.SLOW_SCALE:.2f} "
                f"for {self.SLOW_DURATION_SEC:.1f}s")

    def maybe_trigger_stop_sign(self):
        if self.state != STATE_DRIVING:
            return
        now = time.monotonic()
        if now - self.last_stop_sign_time < self.STOP_SIGN_COOLDOWN:
            return
        if self._find_sign(self.STOP_SIGN_CLASS, self.STOP_SIGN_DIST) is None:
            return
        self.last_stop_sign_time = now
        self.enter_stopped("stop_sign")
    
    def maybe_trigger_left_lane_change(self):
        """green_left 가 멀리(>=LC_TRIGGER_PX) '지속적으로' 보이고 2차선이면 1차선으로 변경."""
        if self.state != STATE_DRIVING:
            self.lc_height_window.clear()
            return
        if self._has_normal_traffic_light():
            self.lc_height_window.clear()
            return
        if self.current_lane != 2 or self.target_lane is not None:
            self.lc_height_window.clear()
            return

        # 이번 프레임 green_left 최대 height (없으면 0 -> 자연 감쇠)
        h_now = 0.0
        for tl in self.latest_objects.get("traffic_lights", []):
            if str(tl.get("state")) != "green_left":
                continue
            try:
                h_now = max(h_now, float(tl.get("height")))
            except (TypeError, ValueError):
                continue
        self.lc_height_window.append(h_now)

        if len(self.lc_height_window) < self.LT_TRIGGER_WINDOW:
            return
        above = sum(1 for x in self.lc_height_window if x >= self.LC_TRIGGER_PX)
        if above >= self.LT_TRIGGER_CONFIRM:
            h_rep = max(self.lc_height_window)
            self.target_lane = 1
            self.lc_height_window.clear()   # 트리거 후 리셋
            self.get_logger().info(f"LEFT-CHANGE: green_left {h_rep:.0f}px -> target_lane=1")

    def maybe_trigger_left_turn(self):
        if self.state != STATE_DRIVING:
            return
        if self._has_normal_traffic_light():     # off-fix 적용된 helper -> 1차선 off는 안 걸림
            self.lt_height_window.clear()
            return
        if self.current_lane != 1:
            self.lt_height_window.clear()
            return
        if time.monotonic() - self.last_left_turn_time < self.LEFT_TURN_COOLDOWN:
            return

        h_now = 0.0
        for tl in self.latest_objects.get("traffic_lights", []):
            if str(tl.get("state")) != "green_left":
                continue
            try:
                h_now = max(h_now, float(tl.get("height")))
            except (TypeError, ValueError):
                continue
        self.lt_height_window.append(h_now)

        if len(self.lt_height_window) < self.LT_TRIGGER_WINDOW:
            return
        above = sum(1 for x in self.lt_height_window if x >= self.LT_TRIGGER_PX)
        if above >= self.LT_TRIGGER_CONFIRM:
            self.enter_left_turn(max(self.lt_height_window))
            self.lt_height_window.clear()

    def enter_left_turn(self, h_px: float):
        self.state = STATE_LEFT_TURN
        self.lt_phase = LT_PHASE_PRE_STOP
        self.lt_phase_start = time.monotonic()
        self.last_left_turn_time = self.lt_phase_start
        self.get_logger().info(
            f"-> LEFT_TURN (green_left {h_px:.0f}px) | PRE_STOP({self.PRE_STOP_SEC:.2f}s) -> "
            f"STRAIGHT({self.LT_STRAIGHT_SEC:.2f}s) -> "
            f"CURVE({self.LT_CURVE_SEC:.2f}s, az={self.LT_CURVE_AZ:+.3f})")

    def enter_lane_change(self):
        if self.current_lane == 2 and self.target_lane == 1:
            self.lc_az_out = +self.LC_ROT_AZ   # 왼쪽 (2->1)
            self.lc_az_in  = -self.LC_ROT_AZ
            direction = "LEFT (2->1)"
        elif self.current_lane == 1 and self.target_lane == 2:
            self.lc_az_out = -self.LC_ROT_AZ   # 오른쪽 (1->2)
            self.lc_az_in  = +self.LC_ROT_AZ
            direction = "RIGHT (1->2)"
        else:
            return
        self.state = STATE_LANE_CHANGE
        self.lc_phase = LC_PHASE_PRE_STOP
        self.lc_phase_start = time.monotonic()
        self.get_logger().info(
            f"-> LANE_CHANGE {direction} | PRE_STOP({self.PRE_STOP_SEC:.2f}s) -> "
            f"ROT_OUT({self.LC_ROT_SEC:.2f}s) -> STRAIGHT({self.LC_STRAIGHT_SEC:.2f}s) -> "
            f"ROT_IN({self.LC_ROT_SEC:.2f}s)")
        
    def advance_lc_phase(self, now: float, next_phase: str):
        self.lc_phase = next_phase
        self.lc_phase_start = now
        self.get_logger().info(f"   LC phase -> {next_phase}")

    # ===== 주기 tick =====
    def tick(self):
        now = time.monotonic()

        # ----- 상태 전이 -----
        if self.state == STATE_STOPPED:
            if now - self.stop_enter_time >= self.STOP_HOLD_SEC:
                self.enter_driving()

        elif self.state == STATE_DRIVING:
            if (self.target_lane is not None
                    and self.current_lane is not None
                    and self.target_lane != self.current_lane):
                self.enter_lane_change()

        elif self.state == STATE_LANE_CHANGE:
            elapsed = now - self.lc_phase_start
            if self.lc_phase == LC_PHASE_PRE_STOP and elapsed >= self.PRE_STOP_SEC:
                self.advance_lc_phase(now, LC_PHASE_ROTATE_OUT)
            elif self.lc_phase == LC_PHASE_ROTATE_OUT and elapsed >= self.LC_ROT_SEC:
                self.advance_lc_phase(now, LC_PHASE_STOP1)
            elif self.lc_phase == LC_PHASE_STOP1 and elapsed >= self.LC_MID_STOP_SEC:
                self.advance_lc_phase(now, LC_PHASE_STRAIGHT)
            elif self.lc_phase == LC_PHASE_STRAIGHT and elapsed >= self.LC_STRAIGHT_SEC:
                self.advance_lc_phase(now, LC_PHASE_STOP2)
            elif self.lc_phase == LC_PHASE_STOP2 and elapsed >= self.LC_MID_STOP_SEC:
                self.advance_lc_phase(now, LC_PHASE_ROTATE_IN)
            elif self.lc_phase == LC_PHASE_ROTATE_IN and elapsed >= self.LC_ROT_SEC:
                self.advance_lc_phase(now, LC_PHASE_STOP3)
            elif self.lc_phase == LC_PHASE_STOP3 and elapsed >= self.LC_MID_STOP_SEC:
                achieved = self.target_lane
                self.target_lane = None
                self.enter_driving(reason=f"(lane change done, target was {achieved})",
                                   assume_lane=achieved)

        elif self.state == STATE_LEFT_TURN:
            elapsed = now - self.lt_phase_start
            if self.lt_phase == LT_PHASE_PRE_STOP and elapsed >= self.PRE_STOP_SEC:
                self.lt_phase = LT_PHASE_STRAIGHT
                self.lt_phase_start = now
                self.get_logger().info("   LT phase -> STRAIGHT")
            elif self.lt_phase == LT_PHASE_STRAIGHT and elapsed >= self.LT_STRAIGHT_SEC:
                self.lt_phase = LT_PHASE_CURVE
                self.lt_phase_start = now
                self.get_logger().info("   LT phase -> CURVE")
            elif self.lt_phase == LT_PHASE_CURVE and elapsed >= self.LT_CURVE_SEC:
                self.enter_driving(reason="(left turn done)")

        # ----- 명령 생성 -----
        speed_scale = 1.0
        if self.state == STATE_STOPPED:
            cmd = CMD_STOP
            angular_z = 0.0
            speed_scale = 0.0
        elif self.state == STATE_LANE_CHANGE:
            if self.lc_phase == LC_PHASE_ROTATE_OUT:
                cmd, angular_z, speed_scale = CMD_FORWARD, self.lc_az_out, 0.0
            elif self.lc_phase == LC_PHASE_STRAIGHT:
                cmd, angular_z, speed_scale = CMD_FORWARD, 0.0, 1.0
            elif self.lc_phase == LC_PHASE_ROTATE_IN:
                cmd, angular_z, speed_scale = CMD_FORWARD, self.lc_az_in, 0.0
            else:  # PRE_STOP / MID_STOP1 / MID_STOP2 -> 정지
                cmd, angular_z, speed_scale = CMD_STOP, 0.0, 0.0

        elif self.state == STATE_LEFT_TURN:
            if self.lt_phase == LT_PHASE_PRE_STOP:
                cmd, angular_z, speed_scale = CMD_STOP, 0.0, 0.0
            elif self.lt_phase == LT_PHASE_STRAIGHT:
                cmd, angular_z, speed_scale = CMD_FORWARD, 0.0, 1.0
            elif self.lt_phase == LT_PHASE_CURVE:
                cmd, angular_z, speed_scale = CMD_FORWARD, self.LT_CURVE_AZ, self.LT_CURVE_SCALE
            else:
                cmd, angular_z, speed_scale = CMD_STOP, 0.0, 0.0
        else:  # DRIVING
            cmd = CMD_FORWARD
            angular_z = self.latest_lane["angular_z"]
            speed_scale = self.SLOW_SCALE if now < self.slow_until else 1.0
            
            if self.traffic_light_blocks() or self.left_traffic_light_blocks():
                cmd = CMD_STOP
                angular_z = 0.0
                speed_scale = 0.0
                
        msg = String()
        msg.data = json.dumps({
            "cmd": cmd,
            "angular_z": round(angular_z, 4),
            "speed_scale": round(speed_scale, 3),
            "state": self.state,
            "lc_phase": self.lc_phase,
            "lt_phase": self.lt_phase,
            "target_lane": self.target_lane,
            "current_lane": self.current_lane,
            "lane": self.latest_lane,
            "objects": self.latest_objects,
        })
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
