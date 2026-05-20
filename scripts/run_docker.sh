#!/bin/bash
xhost +local:docker
docker run -it --rm \
    --env="DISPLAY=$DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd):/workspace" \
    --name="autonomous_dev" \
    my-ros-image:latest
