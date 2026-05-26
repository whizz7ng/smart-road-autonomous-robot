# 시스템 상태/하트비트 (각 노드가 살아있는지 주기적으로 발행)
std_msgs/Header header
string node_name         # 발행 노드 이름 (예: "yolo_detector", "fsm_controller")
bool is_alive            # 노드 살아있음
uint8 error_code         # 0=정상, 1=warning, 2=error, 3=critical
string error_message     # 에러 발생 시 메시지 (정상 시 빈 문자열)