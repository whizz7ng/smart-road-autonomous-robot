# 주행 명령 (제어 노드 -> 시리얼 브릿지 -> ESP32)
std_msgs/Header header
float32 linear_velocity   # m/s, 전진 속도
float32 angular_velocity  # rad/s, 회전 속도 (양수=좌회전)
bool emergency_stop       # 비상정지 플래그
