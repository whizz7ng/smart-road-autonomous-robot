import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge

class Republisher(Node):
    def __init__(self):
        super().__init__('img_republisher')
        self.bridge = CvBridge()
        self.pub = self.create_publisher(CompressedImage, '/camera/image_raw/compressed', 10)
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.cb, qos_profile_sensor_data)
        self.get_logger().info('republisher 시작')

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        out = self.bridge.cv2_to_compressed_imgmsg(frame, dst_format='jpeg')
        out.header = msg.header
        self.pub.publish(out)

rclpy.init()
rclpy.spin(Republisher())

