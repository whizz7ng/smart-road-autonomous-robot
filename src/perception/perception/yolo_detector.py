import sys
import os
import json

# ROS 2 라이브러리 경로를 가상 환경에서도 참조할 수 있도록 강제 추가
sys.path.append('/opt/ros/jazzy/lib/python3.12/site-packages')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String  # Eng B에게 JSON을 보내기 위한 메시지 타입
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2
import numpy as np

class RobotPerceptionNode(Node):
    def __init__(self):
        super().__init__('robot_perception_node')
        self.get_logger().info('통합 자율주행 인식 노드 초기화 시작...')
        self.cv_initialized = False
        
        # [로그 최적화] 상태 저장용 변수
        self.prev_obstacle = None
        self.prev_sign = None
        self.prev_lane_offset = None
        self.prev_stop_line = None
        self.prev_crosswalk = None
        
        try:
            self.bridge = CvBridge()
            
            # 1. YOLO 모델 로드
            model_path = '/ros2_ws/src/perception/weights/best_ncnn_model'
            self.get_logger().info(f'모델 로딩 중: {model_path}')
            
            if not os.path.exists(model_path):
                self.get_logger().error(f'모델 파일을 찾을 수 없습니다: {model_path}')
                return

            self.model = YOLO(model_path, task='detect')
            self.get_logger().info('YOLO 모델 로드 성공!')
            
            # 클래스 이름 확인을 위한 로그 추가
            self.get_logger().info(f'인식 가능한 클래스 목록: {self.model.names}')
            
            # 2. Eng B(제어 담당)에게 보낼 대통합 토픽 발행기 선언
            self.percep_pub = self.create_publisher(String, '/robot_perception', 10)
            
            # 3. 카메라 토픽 구독
            self.subscription = self.create_subscription(
                Image, '/camera/image_raw', self.image_callback, 10)
            
            # 4. 이미지 더블 버퍼링 변수 및 0.1초(10Hz) 주기 연산 타이머 설정
            self.latest_frame = None
            self.timer = self.create_timer(0.1, self.process_timer_callback)
            
            self.get_logger().info('구독 및 0.1초 제어 주기 타이머 설정 완료.55555555555')
            
        except Exception as e:
            self.get_logger().error(f'초기화 중 오류 발생: {e}')

    def image_callback(self, msg):
        try:
            # 원본 영상(30Hz)이 들어오면 최신 프레임 변수에 계속 덮어쓰기 (저장만 수행)
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error(f'영상 수신 중 오류 발생: {e}')

    def process_timer_callback(self):
        # 0.1초마다 한 번씩 실행되어 하드웨어 부하를 줄이는 핵심 루프
        if self.latest_frame is None:
            return

        try:
            # 연산 중 영상이 바뀌는 것을 방지하기 위해 복사본 사용
            frame = self.latest_frame.copy()
            h, w, _ = frame.shape

            # ----------------------------------------------------------------
            # [기능 1] YOLO 데이터 파싱 (장애물, 신호등 세부 상태, Sign 인식)
            # ----------------------------------------------------------------
            # 로그 폭탄을 방지하기 위해 verbose=False 설정
            yolo_results = self.model(frame, verbose=False)
            annotated_frame = yolo_results[0].plot()
            
            # Eng B 전송용 기본 구조체 정의
            yolo_data = {
                "has_obstacle": False,   # 장애물 유무
                "traffic_light": "none", # 신호등 상태 ("none", "red", "green_straight", "green_left")
                "sign": "none"           # 표지판 상태 ("none", "stop", "speed_30", "speed_50" 등)
            }
            
            # 로그용 리스트 초기화 추가
            detected_classes = []
            # [최적화] 상태가 변했는지 체크
            log_msgs = []
            
            # 객체가 검출 되었을 때 분류 처리
            if len(yolo_results[0].boxes) > 0:
                for box in yolo_results[0].boxes:
                    cls_id = int(box.cls[0])
                    cls_name = self.model.names[cls_id] 
                    detected_classes.append(cls_name)
                    
                    # 1) 장애물 감지 (필요 시 클래스 이름 추가)
                    if cls_name in ["person", "obstacle", "box"]:
                        yolo_data["has_obstacle"] = True
                        
                    # 2) 신호등 상태 매핑
                    elif cls_name in ["greenlight", "yellowlight", "redlight", "traffic_light_red", "traffic_left_light_green"]:
                        if cls_name == "redlight" or cls_name == "traffic_light_red":
                            yolo_data["traffic_light"] = "red"
                        elif cls_name == "traffic_left_light_green":
                            yolo_data["traffic_light"] = "green_left"
                        elif cls_name == "greenlight":
                            yolo_data["traffic_light"] = "green_straight"
                    
                    # 3) 표지판 매핑
                    elif cls_name == "stop_sign":
                        yolo_data["sign"] = "stop"
                    elif cls_name == "limit_sign":
                        yolo_data["sign"] = "speed_limit" # 모델이 limit_sign을 인식함

                        

            # ----------------------------------------------------------------
            # [기능 2] OpenCV 도로 차선 처리 (HSV 색상 마스킹 및 오차 계산)
            # ----------------------------------------------------------------
            opencv_data = {
                "lane_offset": 0,       # 정가운데 기준 오차 편차값 (픽셀)
                "is_stop_line": False,  # 정지선 감지 여부
                "is_crosswalk": False   # 횡단보도 감지 여부
            }
            
            # 이미지 하단부 30% 영역을 관심영역(ROI)으로 지정
            roi_y_start = int(h * 0.7)
            #roi = frame[roi_y_start:h, 0:w]
            roi = frame[0:h, 0:w]
            roi_h, roi_w, _ = roi.shape
            roi_center_x = int(roi_w / 2) # ROI 화면의 가로 정가운데

            # BGR에서 HSV 색공간으로 변환
            hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
            
            # 1) 노란색 차선 마스크
            lower_yellow = np.array([15, 50, 50])
            upper_yellow = np.array([35, 255, 255])
            yellow_lane_mask = cv2.inRange(hsv_roi, lower_yellow, upper_yellow)
            
            # 검은색 차선 마스크 추가
            lower_black = np.array([0, 0, 0])
            upper_black = np.array([180, 255, 45])
            black_lane_mask = cv2.inRange(hsv_roi, lower_black, upper_black)
            
            lane_mask = cv2.bitwise_or(yellow_lane_mask, black_lane_mask)

            # 2) 횡단보도 마스크 (검은색(횡단보도) 필터 사용)
            # 명도가 낮고 채도가 낮은 영역(검정~회색)을 검출합니다.
            # 횡단보도는 좀 더 어두운 영역으로만 제한
            lower_crosswalk = np.array([0, 0, 0])
            upper_crosswalk = np.array([180, 255, 30]) # V값을 30으로 더 낮춤
            crosswalk_mask = cv2.inRange(hsv_roi, lower_crosswalk, upper_crosswalk)

            # [수정] 2. 차선 마스크에서 횡단보도 영역을 빼기 (차선 마스크 업데이트)
            # 차선 마스크에서 횡단보도 부분은 제외하여 Lane Mask View에서 횡단보도가 안 나오게 함
            lane_mask = cv2.bitwise_or(yellow_lane_mask, black_lane_mask)
            lane_mask = cv2.subtract(lane_mask, crosswalk_mask)

            # 3. 형태학적 연산 적용 (횡단보도 마스크에만 적용)
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            crosswalk_mask = cv2.morphologyEx(crosswalk_mask, cv2.MORPH_OPEN, kernel)
            crosswalk_mask = cv2.morphologyEx(crosswalk_mask, cv2.MORPH_CLOSE, kernel)
            # ---------------------------------
            
            

            # 로봇 바로 앞부분(예: ROI의 윗부분 20% 영역)만 따로 떼서 무게중심 계산
            look_ahead_roi = lane_mask[0:int(roi_h*0.2), 0:roi_w]
            M = cv2.moments(look_ahead_roi)
            
            if M["m00"] > 500: # 차선 픽셀이 최소 기준 이상 보일 때만
                lane_x = int(M["m10"] / M["m00"])
                # 화면 중심점 대비 실제 차선 중심의 오차 픽셀값 계산
                opencv_data["lane_offset"] = lane_x - roi_center_x
                
                # 디버깅 시각화 표기 (화면중심: 빨간 점, 차선중심: 파란 점)
                cv2.circle(annotated_frame, (roi_center_x, roi_y_start + int(roi_h/2)), 5, (0, 0, 255), -1)
                cv2.circle(annotated_frame, (lane_x, roi_y_start + int(roi_h/2)), 5, (255, 0, 0), -1)
            else:
                opencv_data["lane_offset"] = 0 # 차선 유실 시 정가운데 주행 유지 가이드

                
            # 빨간 실선(정지선) 감지 추가 구현 영역
            lower_red = np.array([0, 70, 50])
            upper_red = np.array([10, 255, 255])
            stop_mask = cv2.inRange(hsv_roi, lower_red, upper_red)
            
            # 횡단보도와 동일한 ROI 방식 적용 (화면 하단 50%~90%)
            margin = int(roi_w * 0.2)
            roi_for_stop = stop_mask[int(roi_h*0.5):int(roi_h*0.9), margin:roi_w-margin]

            # [수정] 픽셀 카운트 기준 설정 (기존 1500 정도가 적당했다면 그대로 사용)
            if cv2.countNonZero(roi_for_stop) > 1500: 
                opencv_data["is_stop_line"] = True
            else:
                opencv_data["is_stop_line"] = False
                

            # 횡단보도 감지
            # 횡단보도 감지 시, 화면의 특정 영역(예: 하단 50%~90%)만 픽셀 카운트
            # 전체 마스크에서 도로 부분만 추출하여 픽셀을 셉니다.
            #roi_for_crosswalk = crosswalk_mask[int(roi_h*0.5):int(roi_h*0.9), 0:roi_w]
            margin = int(roi_w * 0.2)
            roi_for_crosswalk = crosswalk_mask[int(roi_h*0.5):int(roi_h*0.9), margin:roi_w-margin]

            if cv2.countNonZero(roi_for_crosswalk) > 1500:
                opencv_data["is_crosswalk"] = True
            else:
                opencv_data["is_crosswalk"] = False

            # YOLO 변화 체크
            if yolo_data["has_obstacle"] != self.prev_obstacle or yolo_data["sign"] != self.prev_sign:
                log_msgs.append(f'[YOLO] 장애물: {yolo_data["has_obstacle"]} | 표지판: {yolo_data["sign"]}')
                self.prev_obstacle = yolo_data["has_obstacle"]
                self.prev_sign = yolo_data["sign"]
                
            # OpenCV 변화 체크 (Lane Offset은 민감하므로 어느 정도 오차범위 5px 이상일 때만)
            if opencv_data["is_stop_line"] != self.prev_stop_line or \
               opencv_data["is_crosswalk"] != self.prev_crosswalk or \
            (self.prev_lane_offset is None or abs(opencv_data["lane_offset"] - self.prev_lane_offset) > 5):
                log_msgs.append(f'[OpenCV] Lane Offset: {opencv_data["lane_offset"]}px | 정지선: {opencv_data["is_stop_line"]} | 횡단보도: {opencv_data["is_crosswalk"]}')
                self.prev_lane_offset = opencv_data["lane_offset"]
                self.prev_stop_line = opencv_data["is_stop_line"]
                self.prev_crosswalk = opencv_data["is_crosswalk"]

            # [출력] 변화가 있을 때만 로그 출력
            for msg in log_msgs:
                self.get_logger().info(msg)


            # ----------------------------------------------------------------
            # [기능 3] 대통합 JSON 패키징 후 Eng B에게 0.1초마다 송출
            # ----------------------------------------------------------------
            total_perception_data = {
                "yolo": yolo_data,
                "opencv": opencv_data
            }
            
            msg = String()
            msg.data = json.dumps(total_perception_data)
            self.percep_pub.publish(msg)

            # 디버깅 화면 표시 관리
            if not self.cv_initialized:
                cv2.namedWindow("Robot Perception View", cv2.WINDOW_NORMAL)
                cv2.namedWindow("Lane Mask View", cv2.WINDOW_NORMAL)
                cv2.namedWindow("Crosswalk Mask View", cv2.WINDOW_NORMAL)
                self.cv_initialized = True

            cv2.imshow("Robot Perception View", annotated_frame)
            cv2.imshow("Lane Mask View", lane_mask)
            cv2.imshow("Crosswalk Mask View", crosswalk_mask)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info('사용자에 의해 종료됩니다.')
                rclpy.shutdown()

        except Exception as e:
            self.get_logger().error(f'타이머 루프 처리 중 오류 발생: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotPerceptionNode()
    
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