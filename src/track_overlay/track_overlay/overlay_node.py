#!/usr/bin/env python3
import math
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
from PIL import Image

PNG_PATH = "/home/uk24/smart-road-autonomous-robot/src/localization/maps/track_overlay.png"
YAML_PATH = "/home/uk24/smart-road-autonomous-robot/src/localization/maps/track_overlay.yaml"

OFFSET_X = 2.2
OFFSET_Y = -4.2
OFFSET_YAW = -1.65
Z_LEVEL = -0.5
STEP = 6

class OverlayNode(Node):
    def __init__(self):
        super().__init__('track_overlay_node')
        qos = QoSProfile(depth=1)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(Marker, '/track_overlay', qos)
        self.marker = self._build_marker()
        self.marker.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.marker)
        self.get_logger().info('track_overlay published (latched)')

    def _build_marker(self):
        with open(YAML_PATH) as f:
            meta = yaml.safe_load(f)

        origin = meta['origin']
        res    = meta['resolution']
        W      = meta['width_px']
        H      = meta['height_px']

        img = Image.open(PNG_PATH).convert('RGBA')
        pixels = img.load()

        m = Marker()
        m.header.frame_id = 'map'
        m.ns = 'track_overlay'
        m.id = 0
        m.type = Marker.TRIANGLE_LIST
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.scale.x = 1.0
        m.scale.y = 1.0
        m.scale.z = 1.0
        m.frame_locked = True

        ox, oy = origin[0], origin[1]
        cyaw = math.cos(OFFSET_YAW)
        syaw = math.sin(OFFSET_YAW)

        cx_m = ox + (W * res) / 2.0
        cy_m = oy + (H * res) / 2.0

        def transform(wx, wy):
            dx = wx - cx_m
            dy = wy - cy_m
            rx = dx * cyaw - dy * syaw
            ry = dx * syaw + dy * cyaw
            return (cx_m + rx + OFFSET_X, cy_m + ry + OFFSET_Y)

        step = STEP
        for py in range(0, H - step, step):
            for px in range(0, W - step, step):
                wx0 = ox + px * res
                wy0 = oy + (H - py) * res
                wx1 = wx0 + step * res
                wy1 = wy0 - step * res

                r = g = b = 0
                for dy in range(step):
                    for dx in range(step):
                        pix = pixels[px + dx, py + dy]
                        r += pix[0]; g += pix[1]; b += pix[2]
                n = float(step * step)
                color = ColorRGBA(
                    r=float(r) / n / 255.0,
                    g=float(g) / n / 255.0,
                    b=float(b) / n / 255.0,
                    a=1.0,
                )

                t00 = transform(wx0, wy0)
                t10 = transform(wx1, wy0)
                t11 = transform(wx1, wy1)
                t01 = transform(wx0, wy1)

                p00 = Point(x=t00[0], y=t00[1], z=Z_LEVEL)
                p10 = Point(x=t10[0], y=t10[1], z=Z_LEVEL)
                p11 = Point(x=t11[0], y=t11[1], z=Z_LEVEL)
                p01 = Point(x=t01[0], y=t01[1], z=Z_LEVEL)

                m.points += [p00, p10, p11, p00, p11, p01]
                m.colors += [color] * 6

        self.get_logger().info(f'marker built: {len(m.points)} points')
        return m

def main(args=None):
    rclpy.init(args=args)
    node = OverlayNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()