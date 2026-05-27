"""
UGV 키보드 조종 + IMU 텔레메트리 수신 (WSL/Linux 터미널용)
=========================================================
pygame 안 씀. 표준 라이브러리(termios, select)만 사용.
조작:
  W : 직진
  S : 정지
  A : 좌회전 (완만)
  D : 우회전 (완만)
  Q : 제자리 좌회전
  E : 제자리 우회전
  X : 비상정지
  Ctrl+C : 종료

키를 "누르고 있는" 게 아니라, 누를 때마다 상태가 토글되는 방식.
  - W 누르면 직진 시작 → 계속 직진 명령 전송 (워치독 막아줌)
  - S 누르면 정지
  - 다른 방향키 누르면 그쪽으로 전환
"""

import socket
import json
import threading
import time
import sys
import select
import termios
import tty

# ===== 설정 =====
ESP32_IP        = "192.168.0.50"
CMD_PORT        = 4210
TELEMETRY_PORT  = 4211
SEND_RATE_HZ    = 20   # 현재 명령 반복 송신 주기

# ===== 명령 송신 =====
cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
def send_cmd(c: str):
    cmd_sock.sendto(c.encode("ascii"), (ESP32_IP, CMD_PORT))

# ===== 텔레메트리 수신 =====
latest_telemetry = {}
telemetry_lock = threading.Lock()

def telemetry_loop(stop_event):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", TELEMETRY_PORT))
    sock.settimeout(0.5)
    while not stop_event.is_set():
        try:
            data, _ = sock.recvfrom(1024)
        except socket.timeout:
            continue
        try:
            d = json.loads(data.decode("utf-8"))
            with telemetry_lock:
                latest_telemetry.update(d)
        except Exception:
            pass
    sock.close()

# ===== 비차단 키 입력 (Unix/WSL) =====
class NonBlockingInput:
    def __enter__(self):
        self.fd = sys.stdin.fileno()
        self.old_settings = termios.tcgetattr(self.fd)
        tty.setcbreak(self.fd)
        return self
    def __exit__(self, *args):
        termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)

    def get_key(self):
        """키가 눌려있으면 1글자 반환, 아니면 None"""
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None

VALID = set("wasdqex")

def main():
    print(f"Controlling {ESP32_IP}:{CMD_PORT}")
    print(f"Listening telemetry on UDP :{TELEMETRY_PORT}")
    print("Keys: W/A/S/D  Q/E pivot  X estop   Ctrl+C to quit")
    print("-" * 70)

    stop_event = threading.Event()
    t = threading.Thread(target=telemetry_loop, args=(stop_event,), daemon=True)
    t.start()

    current = 'S'
    last_print = 0.0
    interval = 1.0 / SEND_RATE_HZ

    try:
        with NonBlockingInput() as kb:
            while True:
                # 키 입력 처리
                k = kb.get_key()
                if k:
                    kl = k.lower()
                    if kl in VALID:
                        current = kl.upper()

                # 현재 명령 반복 송신 (워치독 유지)
                send_cmd(current)

                # 화면 갱신 (5Hz)
                now = time.time()
                if now - last_print > 0.2:
                    last_print = now
                    with telemetry_lock:
                        d = dict(latest_telemetry)
                    if d:
                        line = (f"CMD={current}  "
                                f"R={d.get('roll', 0):+6.1f}  "
                                f"P={d.get('pitch', 0):+6.1f}  "
                                f"Y={d.get('yaw', 0):+6.1f}  "
                                f"ax={d.get('ax', 0):+5.2f} "
                                f"ay={d.get('ay', 0):+5.2f} "
                                f"az={d.get('az', 0):+5.2f}")
                    else:
                        line = f"CMD={current}  (waiting for telemetry...)"
                    # 같은 줄 갱신
                    sys.stdout.write("\r" + line + " " * 5)
                    sys.stdout.flush()

                time.sleep(interval)
    except KeyboardInterrupt:
        pass
    finally:
        for _ in range(5):
            send_cmd('S')
            time.sleep(0.02)
        stop_event.set()
        print("\nStopped. Bye.")

if __name__ == "__main__":
    main()
