#!/bin/bash
CONTAINER_NAME="smart-road-pi4"
IMAGE_NAME="smart-road-pi4:latest"
WORKSPACE="$(cd "$(dirname "$0")/.." && pwd)"

SERIAL_PORT="/dev/ttyUSB0"
SERIAL_ARGS=""
if [ -e "$SERIAL_PORT" ]; then
    SERIAL_ARGS="--device=$SERIAL_PORT"
    echo "[INFO] ESP32 시리얼 감지: $SERIAL_PORT"
else
    echo "[WARN] $SERIAL_PORT 없음. 시리얼 없이 실행"
fi

if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
    echo "[INFO] 기존 컨테이너 접속"
    docker exec -it $CONTAINER_NAME bash
elif [ "$(docker ps -aq -f name=$CONTAINER_NAME)" ]; then
    echo "[INFO] 중지된 컨테이너 재시작"
    docker start -ai $CONTAINER_NAME
else
    echo "[INFO] 새 컨테이너 생성"
    docker run -it \
        --name $CONTAINER_NAME \
        --net=host \
        --privileged \
        $SERIAL_ARGS \
        --device=/dev/video0 \
        -v "$WORKSPACE:/ros2_ws" \
        -e ROS_DOMAIN_ID=42 \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        $IMAGE_NAME
fi
