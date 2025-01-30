#!/bin/bash

if [ "$#" -lt 2 ]; then
    echo "Usage: $0 <input-path> <output-path> [--resolution <resolution>] [--num-threads <num-threads>]"
    exit 1
fi

INPUT="$1"
OUTPUT_DIR="$2"

shift 2

docker run --rm \
    -v "$INPUT:/app/input" \
    -v "$OUTPUT_DIR:/app/output" \
    point_cloud_viewer /app/point_cloud_viewer/target/release/build_octree \
    /app/input --output-directory /app/output "$@"
