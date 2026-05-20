FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y python3-pip python3-opencv && rm -rf /var/lib/apt/lists/*
RUN pip3 install torch torchvision torchaudio --break-system-packages
WORKDIR /workspace
