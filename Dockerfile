# ============================================================
# proj_dev:latest - 팀원 PC 공용 개발 환경 (amd64, CPU)
# Base: osrf/ros:jazzy-desktop-full (Ubuntu 24.04 + Gazebo + RViz)
# PyTorch: CPU 전용 (학습용 GPU는 별도 환경에서)
# ============================================================
FROM osrf/ros:jazzy-desktop-full

# 비대화형 설치 (apt 프롬프트 방지)
ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=Asia/Seoul

# --- 1) 시스템 빌드 툴 + 편의 도구 ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    vim \
    nano \
    tmux \
    net-tools \
    iputils-ping \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# --- 2) Python & ROS 2 관련 추가 패키지 ---
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pandas \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    ros-jazzy-rosbridge-suite \
    ros-jazzy-teleop-twist-keyboard \
    ros-jazzy-rqt-graph \
    ros-jazzy-rqt-image-view \
    && rm -rf /var/lib/apt/lists/*

# --- 3a) PyTorch CPU 전용 설치 ---
# GPU 학습은 별도 환경에서. dev 도커는 추론/변환(NCNN export)용.
RUN pip3 install --no-cache-dir --break-system-packages \
    --index-url https://download.pytorch.org/whl/cpu \
    torch \
    torchvision \
    torchaudio

# --- 3b) AI / 비전 / HW 통신 패키지 ---
# numpy/matplotlib/scipy/pyyaml/pandas는 apt에 이미 있으므로 제외 (충돌 방지)
RUN pip3 install --no-cache-dir --break-system-packages \
    "ultralytics==8.2.103" \
    opencv-python \
    pyserial \
    ncnn

# --- 4) ROS 2 환경 자동 source (ubuntu 유저용) ---
RUN echo "source /opt/ros/jazzy/setup.bash" >> /home/ubuntu/.bashrc \
    && echo "[ -f /ros2_ws/install/setup.bash ] && source /ros2_ws/install/setup.bash" >> /home/ubuntu/.bashrc \
    && echo "export ROS_DOMAIN_ID=42" >> /home/ubuntu/.bashrc \
    && chown ubuntu:ubuntu /home/ubuntu/.bashrc

# --- 5) 작업 디렉토리 ---
WORKDIR /ros2_ws

# --- 5-1) ubuntu 유저에게 sudo 권한 + /ros2_ws 소유권 부여 ---
RUN echo "ubuntu ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ubuntu \
    && chmod 0440 /etc/sudoers.d/ubuntu \
    && mkdir -p /ros2_ws \
    && chown -R ubuntu:ubuntu /ros2_ws

# --- 5-2) ubuntu 유저로 전환 ---
USER ubuntu

# --- 6) 기본 진입점 ---
CMD ["/bin/bash"]
