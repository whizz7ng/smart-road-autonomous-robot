#!/usr/bin/env python3
"""
ros2_diagnostic - ROS2 통신/TF/Wi-Fi 안정성 한 번에 진단

기능:
  1. /tf, /scan, /odom 의 주파수, 표준편차, timestamp 지연 측정
  2. 라즈베리파이 ping 으로 Wi-Fi 손실률/RTT 측정
  3. 각 항목 점수화 -> 한눈에 좋음/보통/나쁨 평가

사용법:
  source /opt/ros/jazzy/setup.bash
  python3 ros2_diagnostic.py                       # 기본 10초, IP 192.168.0.217
  python3 ros2_diagnostic.py --host 192.168.0.217 --duration 10
"""
import argparse
import subprocess
import threading
import time
import statistics
import re
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

# ===== 평가 기준 =====
# (좋음 임계, 보통 임계). 좋음 <= 임계1, 보통 <= 임계2, 이상 = 나쁨
THRESHOLDS = {
    # hz 편차 (실측/기대 비율의 절대편차)
    "hz_dev": (0.10, 0.30),
    # timestamp - now 의 평균 절대값 [s]
    "delay": (0.10, 0.30),
    # 주파수 표준편차 / 평균
    "jitter_ratio": (0.10, 0.30),
    # ping loss [%]
    "ping_loss": (1.0, 5.0),
    # ping rtt 평균 [ms]
    "ping_rtt": (10.0, 50.0),
    # ping rtt mdev [ms]
    "ping_mdev": (5.0, 20.0),
}


def evaluate(value, thr):
    g, m = thr
    if value <= g:
        return "좋음", "\033[92m"
    if value <= m:
        return "보통", "\033[93m"
    return "나쁨", "\033[91m"


RESET = "\033[0m"


class TopicMonitor(Node):
    """선택한 토픽들의 도착 시각과 timestamp 를 기록."""

    def __init__(self, duration):
        super().__init__("ros2_diagnostic")
        self.duration = duration
        # 토픽별 데이터: { name: {"arrive": [...], "stamp": [...]} }
        self.data = {}

        # /tf 는 best effort 권장
        qos_tf = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_scan = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        qos_odom = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.create_subscription(TFMessage, "/tf",
                                 lambda m: self._cb("/tf", m, is_tf=True),
                                 qos_tf)
        self.create_subscription(LaserScan, "/scan",
                                 lambda m: self._cb("/scan", m),
                                 qos_scan)
        self.create_subscription(Odometry, "/odom",
                                 lambda m: self._cb("/odom", m),
                                 qos_odom)

        self.start_t = time.time()

    def _cb(self, name, msg, is_tf=False):
        now = time.time()
        if name not in self.data:
            self.data[name] = {"arrive": [], "stamp": []}
        self.data[name]["arrive"].append(now)
        if is_tf:
            if not msg.transforms:
                return
            st = msg.transforms[0].header.stamp
        else:
            st = msg.header.stamp
        self.data[name]["stamp"].append(st.sec + st.nanosec * 1e-9)


def analyze_topic(name, info, expected_hz):
    arrive = info["arrive"]
    stamp = info["stamp"]
    n = len(arrive)
    if n < 2:
        return None
    duration = arrive[-1] - arrive[0]
    hz = (n - 1) / duration if duration > 0 else 0.0
    intervals = [arrive[i + 1] - arrive[i] for i in range(n - 1)]
    interval_mean = statistics.mean(intervals)
    interval_std = statistics.stdev(intervals) if n > 2 else 0.0
    jitter_ratio = interval_std / interval_mean if interval_mean > 0 else 0.0

    # timestamp vs 도착시각 차이 (지연)
    delays = [a - s for a, s in zip(arrive, stamp)]
    delay_mean = statistics.mean(delays)
    delay_abs = abs(delay_mean)

    hz_dev = abs(hz - expected_hz) / expected_hz if expected_hz > 0 else 0.0

    return {
        "count": n,
        "hz": hz,
        "hz_dev": hz_dev,
        "jitter_ratio": jitter_ratio,
        "delay_abs": delay_abs,
        "delay_mean": delay_mean,
    }


def run_ping(host, count):
    print(f"\n[Wi-Fi/네트워크] {host} 로 ping {count}회 ...")
    try:
        out = subprocess.run(
            ["ping", "-c", str(count), "-i", "0.2", host],
            capture_output=True, text=True, timeout=count + 10,
        )
        text = out.stdout
    except Exception as ex:
        print(f"  ping 실행 실패: {ex}")
        return None

    # loss
    loss_m = re.search(r"(\d+(?:\.\d+)?)% packet loss", text)
    # rtt
    rtt_m = re.search(
        r"rtt min/avg/max/mdev = "
        r"([\d.]+)/([\d.]+)/([\d.]+)/([\d.]+) ms", text)
    if not loss_m or not rtt_m:
        print("  ping 결과 파싱 실패. 원본 일부:")
        print("  " + text.strip().splitlines()[-1] if text else "")
        return None
    return {
        "loss": float(loss_m.group(1)),
        "rtt_min": float(rtt_m.group(1)),
        "rtt_avg": float(rtt_m.group(2)),
        "rtt_max": float(rtt_m.group(3)),
        "rtt_mdev": float(rtt_m.group(4)),
    }


def print_topic_report(name, expected_hz, res):
    print(f"\n[{name}]   기대 {expected_hz} Hz")
    if res is None:
        print("  데이터 수신 없음.  나쁨")
        return ["나쁨"]
    grades = []

    g, col = evaluate(res["hz_dev"], THRESHOLDS["hz_dev"])
    grades.append(g)
    print(f"  실측 주파수      : {res['hz']:.2f} Hz "
          f"(편차 {res['hz_dev']*100:.1f}%)   {col}{g}{RESET}")

    g, col = evaluate(res["jitter_ratio"], THRESHOLDS["jitter_ratio"])
    grades.append(g)
    print(f"  지터 (std/avg)   : {res['jitter_ratio']*100:.1f}%   "
          f"{col}{g}{RESET}")

    g, col = evaluate(res["delay_abs"], THRESHOLDS["delay"])
    grades.append(g)
    print(f"  timestamp 지연   : {res['delay_mean']*1000:+.1f} ms   "
          f"{col}{g}{RESET}")

    print(f"  수신 메시지 수   : {res['count']}")
    return grades


def print_ping_report(res):
    print()
    print("[Wi-Fi 안정성]")
    if res is None:
        print("  측정 실패.  나쁨")
        return ["나쁨"]
    grades = []
    g, col = evaluate(res["loss"], THRESHOLDS["ping_loss"])
    grades.append(g)
    print(f"  패킷 손실        : {res['loss']:.1f}%   {col}{g}{RESET}")
    g, col = evaluate(res["rtt_avg"], THRESHOLDS["ping_rtt"])
    grades.append(g)
    print(f"  RTT 평균         : {res['rtt_avg']:.1f} ms   "
          f"(min {res['rtt_min']:.1f}, max {res['rtt_max']:.1f})   "
          f"{col}{g}{RESET}")
    g, col = evaluate(res["rtt_mdev"], THRESHOLDS["ping_mdev"])
    grades.append(g)
    print(f"  RTT 편차 (mdev)  : {res['rtt_mdev']:.1f} ms   "
          f"{col}{g}{RESET}")
    return grades


def overall(all_grades):
    counts = {"좋음": 0, "보통": 0, "나쁨": 0}
    for g in all_grades:
        counts[g] += 1
    if counts["나쁨"] > 0:
        verdict = "\033[91m나쁨\033[0m   (조치 필요 항목 있음)"
    elif counts["보통"] >= 2:
        verdict = "\033[93m보통\033[0m   (개선 여지)"
    else:
        verdict = "\033[92m좋음\033[0m   (정상 동작)"
    print()
    print("=" * 60)
    print(f"종합 평가: {verdict}")
    print(f"  좋음 {counts['좋음']}  /  보통 {counts['보통']}  /  나쁨 {counts['나쁨']}")
    print("=" * 60)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--host", default="192.168.0.217",
                        help="라즈베리파이 IP")
    parser.add_argument("--duration", type=int, default=10,
                        help="토픽 측정 시간(초)")
    parser.add_argument("--ping-count", type=int, default=50,
                        help="ping 횟수")
    args = parser.parse_args()

    rclpy.init()
    node = TopicMonitor(args.duration)

    print("=" * 60)
    print(" ROS2 통신/TF/Wi-Fi 안정성 진단")
    print("=" * 60)
    print(f"  대상 호스트      : {args.host}")
    print(f"  토픽 측정 시간   : {args.duration}초")
    print(f"  ping 횟수        : {args.ping_count}회")
    print()
    print(f"[토픽 수신] {args.duration}초 동안 /tf, /scan, /odom 측정 ...")

    # ROS 스핀 별도 스레드
    spin_thread = threading.Thread(
        target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    # ping 은 토픽 측정과 병행
    ping_result = {}
    ping_thread = threading.Thread(
        target=lambda: ping_result.update(
            r=run_ping(args.host, args.ping_count)))
    ping_thread.start()

    # 토픽 측정 대기
    t0 = time.time()
    while time.time() - t0 < args.duration:
        time.sleep(0.5)
        sys.stdout.write(
            f"\r  진행: {int(time.time()-t0)}/{args.duration}초")
        sys.stdout.flush()
    print()

    ping_thread.join()

    # 결과 분석
    expected = {"/tf": 20.0, "/scan": 6.0, "/odom": 20.0}
    all_grades = []
    for name, exp in expected.items():
        info = node.data.get(name)
        res = analyze_topic(name, info, exp) if info else None
        grades = print_topic_report(name, exp, res)
        all_grades.extend(grades)

    grades = print_ping_report(ping_result.get("r"))
    all_grades.extend(grades)

    overall(all_grades)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
