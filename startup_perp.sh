#!/bin/bash

CONTAINER_NAME="smart-road"
IMAGE_NAME="smart-road-pi4:latest"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
WS_SETUP="/ros2_ws/install/setup.bash"
WS_PATH="$HOME/smart-road-autonomous-robot"

log() {
    echo "[$(date '+%H:%M:%S')] $1"
}

log "기존 컨테이너 정리 중..."
docker stop $CONTAINER_NAME 2>/dev/null
docker rm   $CONTAINER_NAME 2>/dev/null

log "컨테이너 시작 중..."
docker run -d \
    --name $CONTAINER_NAME \
    --restart unless-stopped \
    --network host \
    --ipc host \
    --pid host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e ROS_DOMAIN_ID=42 \
    -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v $WS_PATH:/ros2_ws \
    --device=/dev/video0:/dev/video0 \
    --device=/dev/ttyAMA0:/dev/ttyAMA0 \
    $IMAGE_NAME \
    sleep infinity

log "컨테이너 시작 완료. 5초 대기..."
sleep 5

log "rosbridge 설치 확인 중..."
docker exec -u root $CONTAINER_NAME bash -c \
    "apt-get install -y ros-jazzy-rosbridge-suite ros-jazzy-compressed-image-transport -qq 2>/dev/null"

log "republisher.py 복사 중..."
docker exec $CONTAINER_NAME bash -c "cp /ros2_ws/republisher.py /tmp/republisher.py"

log "rosbridge 시작 중..."
docker exec -d $CONTAINER_NAME bash -c \
    "source $ROS_SETUP && \
     ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
     ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
     > /tmp/rosbridge.log 2>&1"

sleep 5

log "perception 시작 중..."
docker exec -d $CONTAINER_NAME bash -c \
    "source $ROS_SETUP && source $WS_SETUP && \
     ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
     ros2 launch perception perception.launch.py \
     > /tmp/perception.log 2>&1"

sleep 8

log "republisher 시작 중..."
docker exec -d $CONTAINER_NAME bash -c \
    "source $ROS_SETUP && source $WS_SETUP && \
     ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
     python3 /tmp/republisher.py \
     > /tmp/republisher.log 2>&1"

sleep 5

log "────────────────────────────────────────"
log "모든 서비스 시작 완료!"
log "rosbridge : ws://$(hostname -I | awk '{print $2}'):9090"
log "────────────────────────────────────────"

log "실행 중인 ROS 노드:"
docker exec $CONTAINER_NAME bash -c \
    "source $ROS_SETUP && source $WS_SETUP && \
     ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
     ros2 node list"
