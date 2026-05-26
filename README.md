# 🚗 Smart Road Autonomous Robot

ROS 2 Jazzy 기반 자율주행 로봇 프로젝트 (2차 프로젝트)

- **시뮬레이션:** Gazebo Harmonic
- **실물:** Raspberry Pi 4 + ESP32 + USB Camera + LiDAR
- **AI:** YOLOv8n + NCNN + INT8 (RPi4 CPU 추론)

---

## 📋 목차

1. [팀원 빠른 시작](#-팀원-빠른-시작-quick-start)
2. [폴더 구조](#-폴더-구조)
3. [개발 워크플로우](#-개발-워크플로우-git)
4. [통신 인터페이스 (Topic)](./TOPIC_SPEC.md)
5. [트러블슈팅](#-트러블슈팅)
6. [Eng A 전용](#-eng-a-전용-이미지-재빌드--배포)

---

## 🚀 팀원 빠른 시작 (Quick Start)

### 사전 요구사항

- WSL2 (Ubuntu) 또는 Ubuntu 24.04
- Docker Engine 또는 Docker Desktop
- Git SSH 키 등록 완료
- 디스크 여유 공간 최소 10GB

### 1단계: 레포지토리 clone

    git clone git@github.com:whizz7ng/smart-road-autonomous-robot.git
    cd smart-road-autonomous-robot

### 2단계: 마스터 도커 이미지 pull

    docker pull kws86dockerhub/smart-road-dev:latest

약 5~10분 소요 (1.7GB 다운로드)

### 3단계: 컨테이너 실행

    ./scripts/run.sh

첫 실행 시 컨테이너 자동 생성. 이후 같은 명령으로 재진입 가능.

### 4단계: 동작 확인 (컨테이너 안에서)

    # ROS 2 동작 확인
    ros2 pkg list | wc -l

    # Python AI 스택 확인
    python3 -c "import torch, cv2, serial, ncnn; from ultralytics import YOLO; print('All OK')"

    # GUI 테스트
    rviz2
    gz sim empty.sdf

GUI 창이 호스트 PC에 정상 표출되면 환경 개통 완료.

---

## 📁 폴더 구조

    smart-road-autonomous-robot/
    ├── Dockerfile              # 마스터 이미지 빌드 정의 (Eng A 관리)
    ├── README.md               # 본 문서
    ├── .gitignore
    ├── config/                 # 로봇 파라미터, 설정값
    ├── data/                   # 로그, 데이터셋 (Git 제외)
    ├── scripts/
    │   └── run.sh              # 컨테이너 실행 스크립트
    └── src/                    # ROS 2 패키지 소스
        ├── sensors/            # 카메라, 라이다 등 센서 노드
        ├── control/            # 주행 제어 로직
        └── perception/         # YOLO 추론, 차선 인식

---

## 🌿 개발 워크플로우 (Git)

### 기본 규칙

- main 브랜치 직접 푸시 금지
- 모든 작업은 feat/<본인>-<기능> 브랜치에서 진행
- PR(Pull Request) → PM(Leader) 리뷰 → Merge

### 작업 절차

    # 1. 최신 상태 동기화
    git checkout main
    git pull origin main

    # 2. 작업 브랜치 생성
    git checkout -b feat/memberA-camera

    # 3. 작업 후 커밋 & 푸시
    git add .
    git commit -m "feat: 카메라 노드 기능 구현"
    git push origin feat/memberA-camera

푸시 후 GitHub 웹페이지에 "Compare & pull request" 버튼 표시됨 → 클릭하여 PR 생성 → Leader 승인 후 Merge.

---

## 🔧 트러블슈팅

### Q1. run.sh: Permission denied

    chmod +x scripts/run.sh

### Q2. GUI 창이 안 뜸 (Could not connect to display)

호스트 터미널(컨테이너 밖)에서:

    xhost +local:docker
    echo $DISPLAY

### Q3. 컨테이너 안에서 ros2 명령 못 찾음

    source /opt/ros/jazzy/setup.bash

### Q4. 컨테이너 중복 에러

    docker rm -f proj_dev
    ./scripts/run.sh

### Q5. 호스트-컨테이너 ROS 2 통신 안 됨

- 모든 환경에서 ROS_DOMAIN_ID=42 통일 (이미지 기본값)
- --net=host 옵션 필수 (run.sh 포함됨)

---

## 🛠 Eng A 전용 (이미지 재빌드 / 배포)

### 마스터 이미지 재빌드

    docker build -t proj_dev:latest .

### Docker Hub 배포

    docker tag proj_dev:latest kws86dockerhub/smart-road-dev:latest
    docker push kws86dockerhub/smart-road-dev:latest

### 베이스 이미지 정보

- Base: osrf/ros:jazzy-desktop-full
- OS: Ubuntu 24.04 LTS
- ROS 2: Jazzy Jalisco (Desktop Full)
- Architecture: amd64 (RPi용 arm64는 별도 빌드 예정)

### 포함된 주요 패키지

- ROS 2: jazzy-desktop-full (Gazebo Harmonic, RViz2, rosbridge, cv_bridge)
- AI/Vision: torch (CPU), ultralytics 8.2.103, opencv-python, ncnn
- HW 통신: pyserial
- 빌드 도구: colcon, build-essential, cmake

---

## 📌 환경 변수

- ROS_DOMAIN_ID = 42 (팀 전용 DDS 도메인)
- TZ = Asia/Seoul (컨테이너 시간대)

---

## 👥 팀

- Leader (PM): AI 인지/모델링, 전체 일정 관리
- Engineer A : 도커/ROS 시스템, 시리얼 통신
- Engineer B : 자율주행 제어 (PID/FSM), ESP32 펌웨어
- Engineer C : Gazebo, 실물 HW, 관제 시스템
