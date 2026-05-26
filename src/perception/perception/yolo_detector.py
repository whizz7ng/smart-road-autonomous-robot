import sys
import os

# ROS 2 라이브러리 경로를 가상 환경에서도 참조할 수 있도록 강제 추가
sys.path.append('/opt/ros/jazzy/lib/python3.12/site-packages')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YoloDetector(Node):
    def __init__(self):
        super().__init__('yolo_detector')
        self.get_logger().info('노드 초기화 시작...')
        
        try:
            self.bridge = CvBridge()
            
            # 모델 경로 설정
            model_path = os.path.expanduser('/ros2_ws/src/perception/weights/best.pt')
            self.get_logger().info(f'모델 로딩 중: {model_path}')
            
            if not os.path.exists(model_path):
                self.get_logger().error(f'모델 파일을 찾을 수 없습니다: {model_path}')
                return

            self.model = YOLO(model_path)
            self.get_logger().info('YOLO 모델 로드 성공!')
            
            # 카메라 토픽 구독
            self.subscription = self.create_subscription(
                Image, '/camera/image_raw', self.image_callback, 10)
            self.get_logger().info('토픽 구독 설정 완료. 영상 수신 대기 중...')
            
        except Exception as e:
            self.get_logger().error(f'초기화 중 오류 발생: {e}')

    def image_callback(self, msg):
        try:
            # 1. ROS 이미지를 OpenCV BGR 이미지로 변환
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 2. YOLO 추론
            results = self.model(cv_image)
            
            # 3. 결과 그리기
            annotated_frame = results[0].plot()
            
            # 4. 창 제목을 명확히 하고, 창 생성/업데이트 관리
            cv2.namedWindow("YOLO Detection", cv2.WINDOW_NORMAL)
            cv2.imshow("YOLO Detection", annotated_frame)
            
            # 5. 대기 시간 확보 (20Hz 정도면 50ms가 적절합니다)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('사용자에 의해 종료됩니다.')
                rclpy.shutdown()
                
        except Exception as e:
            self.get_logger().error(f'영상 처리 중 오류 발생: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YoloDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()