#!/bin/bash

set -e

cd "$(dirname "$0")"

xhost +local:docker

CARTOGRAPHER_PREFIX="/opt/ros/humble/share/cartographer_ros"

mkdir -p data

docker run -it \
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="${PWD}/data:/data" \
    --volume="${PWD}/configuration_files:${CARTOGRAPHER_PREFIX}/configuration_files" \
    --volume="${PWD}/launch:${CARTOGRAPHER_PREFIX}/launch" \
    --device=/dev/dri:/dev/dri \
    cartographer
