import rclpy
from rclpy.node import Node

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.get_logger().info('YOLO Detector 노드가 시작되었습니다!')
        # 여기에 추후 YOLO 모델 로드 코드가 들어갑니다.

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()