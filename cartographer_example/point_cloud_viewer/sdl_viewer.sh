#!/bin/bash

if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <octree directory>"
    exit 1
fi

OCTREE_DIR="$1"

docker run --rm \
    -v "$OCTREE_DIR:/app/octree_dir" \
    --env="DISPLAY" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --device=/dev/dri:/dev/dri \
    point_cloud_viewer /app/point_cloud_viewer/target/release/sdl_viewer \
    /app/octree_dir
