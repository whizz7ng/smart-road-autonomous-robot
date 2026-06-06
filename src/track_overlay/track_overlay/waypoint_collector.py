#!/usr/bin/env python3
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped

SAVE_PATH = "/home/uk24/smart-road-autonomous-robot/src/localization/maps/waypoints.yaml"

class WaypointCollector(Node):
    def __init__(self):
        super().__init__('waypoint_collector')
        self.sub = self.create_subscription(
            PointStamped, '/clicked_point', self.cb, 10)
        self.points = []
        self.get_logger().info('waypoint_collector ready - RViz Publish Point로 클릭하세요')

    def cb(self, msg):
        x = round(msg.point.x, 4)
        y = round(msg.point.y, 4)
        self.points.append({'x': x, 'y': y})
        idx = len(self.points)
        self.get_logger().info(f'[{idx}] x={x}, y={y}  (총 {idx}개)')
        self._save()

    def _save(self):
        data = {'waypoints': self.points}
        with open(SAVE_PATH, 'w') as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)

def main(args=None):
    rclpy.init(args=args)
    node = WaypointCollector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
