#!/bin/bash
# ============================================================
# proj_dev 컨테이너 실행 스크립트
# 사용법: ./scripts/run.sh
# ============================================================

set -e

IMAGE_NAME="kws86dockerhub/smart-road-dev:latest"
CONTAINER_NAME="proj_dev"
WORKSPACE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# --- X11 GUI 권한 허용 ---
xhost +local:docker > /dev/null 2>&1 || true

# --- 기존 컨테이너가 있으면 접속, 없으면 새로 생성 ---
if [ "$(docker ps -aq -f name=^${CONTAINER_NAME}$)" ]; then
    if [ "$(docker ps -q -f name=^${CONTAINER_NAME}$)" ]; then
        echo "[INFO] 기존 컨테이너에 접속합니다."
        docker exec -it ${CONTAINER_NAME} /bin/bash
    else
        echo "[INFO] 중지된 컨테이너를 시작합니다."
        docker start ${CONTAINER_NAME}
        docker exec -it ${CONTAINER_NAME} /bin/bash
    fi
else
    echo "[INFO] 새 컨테이너를 생성합니다."
    docker run -it \
        --name ${CONTAINER_NAME} \
        --net=host \
        --ipc=host \
        --pid=host \
        --privileged \
        -e DISPLAY=${DISPLAY} \
        -e QT_X11_NO_MITSHM=1 \
        -e XDG_RUNTIME_DIR=/tmp/runtime-root \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v ${WORKSPACE_DIR}:/ros2_ws \
        -v /dev:/dev \
        ${IMAGE_NAME} \
        /bin/bash
fi
