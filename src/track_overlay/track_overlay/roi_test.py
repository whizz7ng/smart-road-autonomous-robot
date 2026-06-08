#!/usr/bin/env python3
import math
from collections import deque
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException, ConnectivityException

ROI_FWD_MIN = 0.10
ROI_FWD_MAX = 1.0
ROI_SIDE_MIN = -0.33
ROI_SIDE_MAX = 0.33
ROI_MIN_POINTS = 3

GRID_SIZE = 0.15          # map 격자 크기(m)
STATIC_WINDOW = 5         # 최근 N프레임
STATIC_HITS = 4           # N프레임 중 M번 이상 점유 = 정적

DYNAMIC_MIN_CELLS = 2     # 동적 판정 최소 동적 셀 수 (가장자리 1셀 깜빡임 무시)
DYNAMIC_HOLD = 8          # 켜진 뒤 유지 프레임 (~0.8s @10Hz)

class RoiTest(Node):
    def __init__(self):
        super().__init__('roi_test')
        self.sub = self.create_subscription(LaserScan, '/scan', self.cb, 10)
        self.box_pub = self.create_publisher(Marker, '/roi_box', 10)
        self.pts_pub = self.create_publisher(Marker, '/roi_points_map', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # 정적/동적 판정 상태
        # 정적/동적 판정 상태
        self.streak = {}              # 칸별 연속 점유 카운트
        self.prev_occupied = set()    # 직전 프레임 점유 격자
        self.last_dynamic = False     # FSM 전달용 결과
        self.get_logger().info('roi_test (map frame) ready')
        self.dyn_hold = 0

    def _get_tf(self):
        try:
            return self.tf_buffer.lookup_transform(
                'odom', 'base_laser', rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except (LookupException, ExtrapolationException, ConnectivityException):
            return None

    def publish_box(self):
        m = Marker()
        m.header.frame_id = 'base_laser'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'roi_box'
        m.id = 0
        m.type = Marker.LINE_STRIP
        m.action = Marker.ADD
        m.scale.x = 0.02
        m.color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
        m.pose.orientation.w = 1.0
        # ROI(fwd,side)->base_laser(x=side, y=-fwd)
        corners = [
            (ROI_FWD_MIN, ROI_SIDE_MIN), (ROI_FWD_MAX, ROI_SIDE_MIN),
            (ROI_FWD_MAX, ROI_SIDE_MAX), (ROI_FWD_MIN, ROI_SIDE_MAX),
            (ROI_FWD_MIN, ROI_SIDE_MIN),
        ]
        for fwd, side in corners:
            m.points.append(Point(x=side, y=-fwd, z=0.0))
        self.box_pub.publish(m)

    def check_dynamic_obstacle(self, occupied):
        """현재 프레임 점유 격자(occupied: set)를 받아 동적 장애물 유무 반환.
        - 정적: STATIC_HITS 프레임 '연속' 점유된 칸 (중간에 비면 리셋)
        - 동적: 정적이 아닌 칸이 직전 프레임에도 점유(2프레임 연속)
        """
        # 연속 점유 카운트 갱신: 이번에 점유된 칸은 +1, 안 된 칸은 리셋(삭제)
        new_streak = {}
        for cell in occupied:
            new_streak[cell] = self.streak.get(cell, 0) + 1
        for cell, n in self.streak.items():
            if cell not in occupied and n - 1 > 0:
                new_streak[cell] = n - 1   # 안 보인 칸: 리셋 대신 1 감쇠
        self.streak = new_streak

        # 정적 칸: 연속 STATIC_HITS 이상 점유
        static_cells = {cell for cell, n in self.streak.items() if n >= STATIC_HITS}

        # 동적 후보: 정적이 아니면서 직전 프레임에도 있던 칸
        dynamic_cells = {
            cell for cell in occupied
            if cell not in static_cells and cell in self.prev_occupied
        }

        # 상태 갱신
        self.prev_occupied = occupied
        n_dyn = len(dynamic_cells)
        if n_dyn >= DYNAMIC_MIN_CELLS:
            self.dyn_hold = DYNAMIC_HOLD          # 강한 증거 -> 유지 풀충전
        elif n_dyn >= 1 and self.dyn_hold > 0:
            self.dyn_hold = DYNAMIC_HOLD          # 동적 진행 중 약한 증거 -> 연장
        elif self.dyn_hold > 0:
            self.dyn_hold -= 1                    # 증거 없음 -> 감쇠
        self.last_dynamic = self.dyn_hold > 0
        return self.last_dynamic, n_dyn, len(static_cells)

    def cb(self, scan):
        self.publish_box()
        tf = self._get_tf()
        if tf is None:
            return
        tx = tf.transform.translation.x
        ty = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0*(q.w*q.z+q.x*q.y), 1.0-2.0*(q.y*q.y+q.z*q.z))
        cy, sy = math.cos(yaw), math.sin(yaw)

        pts = Marker()
        pts.header.frame_id = 'map'
        pts.header.stamp = self.get_clock().now().to_msg()
        pts.ns = 'roi_points_map'
        pts.id = 0
        pts.type = Marker.POINTS
        pts.action = Marker.ADD
        pts.scale.x = 0.05
        pts.scale.y = 0.05
        pts.color = ColorRGBA(r=1.0, g=0.0, b=0.0, a=1.0)
        pts.pose.orientation.w = 1.0

        count = 0
        occupied = set()
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
                    # base_laser -> map 변환
                    mx = tx + (x*cy - y*sy)
                    my = ty + (x*sy + y*cy)
                    pts.points.append(Point(x=mx, y=my, z=0.0))
                    # map 좌표 -> 격자 칸
                    gx = math.floor(mx / GRID_SIZE)
                    gy = math.floor(my / GRID_SIZE)
                    occupied.add((gx, gy))
            angle += scan.angle_increment
        self.pts_pub.publish(pts)

        obstacle = count >= ROI_MIN_POINTS
        dynamic, n_dyn, n_static = self.check_dynamic_obstacle(occupied)
        self.get_logger().info(
            f'ROI points={count} obstacle={obstacle} '
            f'dynamic={dynamic} (dyn_cells={n_dyn} static_cells={n_static})')

def main(args=None):
    rclpy.init(args=args)
    node = RoiTest()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
