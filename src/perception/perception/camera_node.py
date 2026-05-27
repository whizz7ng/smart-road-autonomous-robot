#!/usr/bin/env python3
import sys
# ROS 2 라이브러리 경로를 가상 환경에서도 참조할 수 있도록 강제 추가
sys.path.append('/opt/ros/jazzy/lib/python3.12/site-packages')

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        # --- 파라미터 선언 (launch 파일이나 CLI에서 변경 가능) ---
        self.declare_parameter('device_id', 1)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('topic_name', '/camera/image_raw')
        self.declare_parameter('frame_id', 'camera_link')

        device_id = self.get_parameter('device_id').value
        self.frame_width = self.get_parameter('frame_width').value
        self.frame_height = self.get_parameter('frame_height').value
        fps = self.get_parameter('fps').value
        topic_name = self.get_parameter('topic_name').value
        self.frame_id = self.get_parameter('frame_id').value

        # --- 카메라 초기화 ---
        self.get_logger().info(f'카메라 초기화 시도 (device={device_id})...')
        self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)

        if not self.cap.isOpened():
            self.get_logger().error(
                f'카메라(/dev/video{device_id})를 열 수 없습니다. '
                f'연결 상태와 권한을 확인하세요.'
            )
            raise RuntimeError(f'카메라 열기 실패: device_id={device_id}')

        # MJPG 포맷이 USB 웹캠에서 대역폭 효율이 좋음
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, fps)

        # 실제 설정된 값 로깅 (요청값과 다를 수 있음)
        actual_w = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self.cap.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'카메라 설정 완료: {actual_w}x{actual_h} @ {actual_fps:.1f}fps'
        )

        # --- 퍼블리셔 설정 ---
        # 실시간 영상은 BEST_EFFORT가 일반적으로 적합 (지연 누적 방지)
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher_ = self.create_publisher(Image, topic_name, qos)

        # --- 타이머 설정 ---
        self.bridge = CvBridge()
        self.capture_fail_count = 0
        timer_period = 1.0 / fps
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'카메라 노드 시작 완료. 토픽 발행 중: {topic_name}'
        )

    def timer_callback(self):
        ret, frame = self.cap.read()

        if not ret or frame is None:
            self.capture_fail_count += 1
            if self.capture_fail_count == 1 or self.capture_fail_count % 30 == 0:
                self.get_logger().warning(
                    f'프레임 캡처 실패 (누적 {self.capture_fail_count}회)'
                )
            return

        if self.capture_fail_count > 0:
            self.get_logger().info('카메라 캡처가 정상화되었습니다.')
            self.capture_fail_count = 0

        # 카메라가 요청 해상도를 안 줄 때만 resize
        if frame.shape[1] != self.frame_width or frame.shape[0] != self.frame_height:
            frame = cv2.resize(
                frame,
                (self.frame_width, self.frame_height),
                interpolation=cv2.INTER_NEAREST,
            )

        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.publisher_.publish(msg)

    def cleanup(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        self.get_logger().info('카메라 자원을 안전하게 해제했습니다.')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CameraNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().info('사용자에 의해 카메라 노드가 종료됩니다.')
    except Exception as e:
        print(f'[camera_node] 시작 실패: {e}', file=sys.stderr)
    finally:
        if node is not None:
            node.cleanup()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
