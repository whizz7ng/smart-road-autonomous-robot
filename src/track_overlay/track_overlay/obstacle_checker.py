#!/usr/bin/env python3
"""ROI 기반 정적/동적 장애물 판정 모듈 (FSM 통합용).

사용 예:
    from track_overlay.obstacle_checker import ObstacleChecker

    self.checker = ObstacleChecker(self)          # self = rclpy Node
    ...
    def scan_cb(self, scan):
        res = self.checker.update(scan)
        if res.dynamic:
            # 동적 장애물 -> 정지
        elif res.obstacle:
            # 정적만 -> 정책대로 (무시/주행)

반환값 ObstacleResult:
    obstacle (bool) : ROI 안 장애물 유무 (정적+동적 전부). TF 없어도 base_laser 기준으로 보장
    dynamic  (bool) : 동적 장애물 유무. map TF 필요 -> TF 실패 시 항상 False
    count    (int)  : ROI 안 검출 점 개수
    valid    (bool) : map TF 정상 여부. False면 dynamic 판정 신뢰 불가(obstacle은 유효)
"""
import math
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException, ConnectivityException
import rclpy
from rclpy.duration import Duration

# --- roi_test.py 와 동일 규격 ---
ROI_FWD_MIN = 0.1
ROI_FWD_MAX = 1.0
ROI_SIDE_MIN = -0.33
ROI_SIDE_MAX = 0.33
ROI_MIN_POINTS = 3

GRID_SIZE = 0.15          # map 격자 크기(m)
STATIC_HITS = 4           # 연속 N프레임 점유 = 정적
DYNAMIC_MIN_CELLS = 2     # 트리거: 이 이상이면 동적 ON
DYNAMIC_HOLD = 8          # 켜진 뒤 유지 프레임 (~0.8s @10Hz)

class ObstacleResult:
    __slots__ = ('obstacle', 'dynamic', 'count', 'valid')

    def __init__(self, obstacle, dynamic, count, valid):
        self.obstacle = obstacle
        self.dynamic = dynamic
        self.count = count
        self.valid = valid



class ObstacleChecker:
    def __init__(self, node, tf_buffer=None):
        """node: tf 조회 및 시간 기준용 rclpy Node.
        tf_buffer: 이미 만든 Buffer가 있으면 재사용(없으면 내부 생성)."""
        self._node = node
        if tf_buffer is not None:
            self.tf_buffer = tf_buffer
            self._own_listener = None
        else:
            self.tf_buffer = Buffer()
            self._own_listener = TransformListener(self.tf_buffer, node)
        # 정적/동적 판정 상태
        self.streak = {}              # 칸별 연속 점유 카운트
        self.prev_occupied = set()    # 직전 프레임 점유 격자
        self.dyn_hold = 0

    def _get_tf(self):
        try:
            return self.tf_buffer.lookup_transform(
                'odom', 'base_laser', rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except (LookupException, ExtrapolationException, ConnectivityException):
            return None

    def update(self, scan):
        """LaserScan 1프레임을 받아 ObstacleResult 반환."""
        tf = self._get_tf()

        # 1) ROI 점 개수(obstacle) 집계 - TF 없어도 base_laser 기준으로 보장
        count = 0
        roi_pts = []   # (x, y) base_laser 좌표 보관 (TF 있을 때 map 변환용)
        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min < r < scan.range_max:
                x = r * math.cos(angle)
                y = r * math.sin(angle)
                fwd = -y
                side = x
                if (ROI_FWD_MIN < fwd < ROI_FWD_MAX and
                        ROI_SIDE_MIN < side < ROI_SIDE_MAX):
                    count += 1
                    roi_pts.append((x, y))
            angle += scan.angle_increment

        obstacle = count >= ROI_MIN_POINTS

        # 2) TF 없으면 dynamic 판정 불가 -> obstacle만 반환, 상태는 건드리지 않음
        if tf is None:
            return ObstacleResult(obstacle, False, count, False)

        # 3) map 변환 후 격자 점유 집계
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))
        cy, sy = math.cos(yaw), math.sin(yaw)

        occupied = set()
        for x, y in roi_pts:
            mx = tx + (x*cy - y*sy)
            my = ty + (x*sy + y*cy)
            gx = math.floor(mx / GRID_SIZE)
            gy = math.floor(my / GRID_SIZE)
            occupied.add((gx, gy))

        # 4) 정적/동적 판정 (roi_test 와 동일 로직)
        new_streak = {}
        for cell in occupied:
            new_streak[cell] = self.streak.get(cell, 0) + 1
        for cell, n in self.streak.items():
            if cell not in occupied and n - 1 > 0:
                new_streak[cell] = n - 1   # 안 보인 칸: 리셋 대신 1 감쇠
        self.streak = new_streak

        static_cells = {cell for cell, n in self.streak.items() if n >= STATIC_HITS}
        dynamic_cells = {
            cell for cell in occupied
            if cell not in static_cells and cell in self.prev_occupied
        }
        self.prev_occupied = occupied
        n_dyn = len(dynamic_cells)
        if n_dyn >= DYNAMIC_MIN_CELLS:
            self.dyn_hold = DYNAMIC_HOLD          # 강한 증거 -> 유지 풀충전
        elif n_dyn >= 1 and self.dyn_hold > 0:
            self.dyn_hold = DYNAMIC_HOLD          # 동적 진행 중 약한 증거 -> 연장
        elif self.dyn_hold > 0:
            self.dyn_hold -= 1                    # 증거 없음 -> 감쇠
        dynamic = self.dyn_hold > 0
        return ObstacleResult(obstacle, dynamic, count, True)
