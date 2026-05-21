# RPi <-> ESP32 양방향 패킷 (시리얼 통신)
std_msgs/Header header
uint8 packet_type         # 0=cmd, 1=feedback, 2=status, 3=error
float32 left_motor        # 좌측 모터 속도(cmd) 또는 엔코더(feedback)
float32 right_motor       # 우측 모터 속도(cmd) 또는 엔코더(feedback)
float32 imu_yaw           # IMU yaw 각도 (실물에서만 사용)
uint8 status_code         # 0=ok, 그 외 에러 코드
