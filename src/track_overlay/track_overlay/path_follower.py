#!/usr/bin/env python3
import os
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener, LookupException, ExtrapolationException, ConnectivityException

WP_PATH = os.environ.get("WAYPOINTS_PATH",
    "/home/uk24/smart-road-autonomous-robot/src/localization/maps/waypoints.yaml")

LINEAR_SPEED   = 0.23   # m/s 전진속도
LOOKAHEAD      = 0.40   # m 전방주시거리
GOAL_TOL       = 0.10   # m 도착판정
INTERP_STEP    = 0.05   # m 보간간격
MAX_ANGULAR    = 1.5    # rad/s 회전제한
PUBLISH_HZ     = 20.0

class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')
        self.pub = self.create_publisher(TwistStamped, '/nav_cmd', 10)
        self.path_pub = self.create_publisher(Path, '/nav_path', 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.path = self._load_and_interp()
        self.path_msg = self._build_path_msg()
        self.finished = False
        self.timer = self.create_timer(1.0 / PUBLISH_HZ, self.loop)
        self.get_logger().info(f'path_follower ready - {len(self.path)} interp points')

    def _load_and_interp(self):
        with open(WP_PATH) as f:
            data = yaml.safe_load(f)
        raw = [(p['x'], p['y']) for p in data['waypoints']]
        path = []
        for i in range(len(raw) - 1):
            x0, y0 = raw[i]
            x1, y1 = raw[i + 1]
            seg = math.hypot(x1 - x0, y1 - y0)
            n = max(1, int(seg / INTERP_STEP))
            for k in range(n):
                t = k / n
                path.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
        path.append(raw[-1])
        return path

    def _build_path_msg(self):
        pm = Path()
        pm.header.frame_id = 'map'
        for (x, y) in self.path:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = x
            ps.pose.position.y = y
            ps.pose.orientation.w = 1.0
            pm.poses.append(ps)
        return pm

    def _get_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                'map', 'base_link', rclpy.time.Time(),
                timeout=Duration(seconds=0.1))
        except (LookupException, ExtrapolationException, ConnectivityException):
            return None
        x = tf.transform.translation.x
        y = tf.transform.translation.y
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (x, y, yaw)

    def _find_target(self, rx, ry):
        # 가장 가까운 점 인덱스
        best_i = 0
        best_d = float('inf')
        for i, (px, py) in enumerate(self.path):
            d = math.hypot(px - rx, py - ry)
            if d < best_d:
                best_d = d
                best_i = i
        # 그 이후로 lookahead 넘는 첫 점
        for i in range(best_i, len(self.path)):
            px, py = self.path[i]
            if math.hypot(px - rx, py - ry) >= LOOKAHEAD:
                return self.path[i]
        return self.path[-1]

    def loop(self):
        # 경로 시각화 발행
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_pub.publish(self.path_msg)

        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'

        if self.finished:
            self.pub.publish(msg)  # 0속도
            return

        pose = self._get_pose()
        if pose is None:
            self.pub.publish(msg)
            return
        rx, ry, ryaw = pose

        # 마지막 점 도착 판정
        gx, gy = self.path[-1]
        if math.hypot(gx - rx, gy - ry) < GOAL_TOL:
            self.finished = True
            self.get_logger().info('goal reached - stop')
            self.pub.publish(msg)
            return

        tx, ty = self._find_target(rx, ry)
        # 로봇 좌표계로 목표점 변환
        dx = tx - rx
        dy = ty - ry
        lx = math.cos(-ryaw) * dx - math.sin(-ryaw) * dy
        ly = math.sin(-ryaw) * dx + math.cos(-ryaw) * dy

        # Pure Pursuit 곡률
        ld2 = lx * lx + ly * ly
        if ld2 < 1e-6:
            curvature = 0.0
        else:
            curvature = 2.0 * ly / ld2

        v = LINEAR_SPEED
        w = v * curvature
        w = max(-MAX_ANGULAR, min(MAX_ANGULAR, w))

        msg.twist.linear.x = v
        msg.twist.angular.z = w
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()