# 로봇 FSM 상태 (제어 노드 -> 관제 시스템)
std_msgs/Header header
string fsm_state          # "IDLE", "DRIVING", "STOPPING", "TURNING", "EMERGENCY"
float32 current_speed     # 현재 속도 (m/s)
string current_action     # "straight", "left_turn", "wait_pedestrian" 등
bool obstacle_detected    # 장애물 감지 여부
