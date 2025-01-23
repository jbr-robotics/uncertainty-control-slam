#!/bin/bash

# Check if the correct number of arguments is provided
if [ "$#" -ne 3 ]; then
    echo "Usage: $0 <calibration_path> <dataset_path> <output_path>"
    exit 1
fi

# Assign arguments to variables
CALIBRATION_PATH=$(realpath $1)
DATASET_PATH=$(realpath $2)
OUTPUT_PATH=$(realpath $3)

# Get the parent directory of the output path
OUTPUT_DIR=$(dirname "$OUTPUT_PATH")

# Run the Docker container
docker run --rm \
    --entrypoint "" \
    -v "$CALIBRATION_PATH":/data/calibration:ro \
    -v "$DATASET_PATH":/data/dataset:ro \
    -v "$OUTPUT_DIR":/data/output \
    kitti_to_rosbag:latest \
    /bin/bash -c "source /opt/ros/noetic/setup.bash && \
                  source /root/catkin_ws/devel/setup.bash && \
                  rosrun kitti_to_rosbag kitti_rosbag_converter \
                  /data/calibration /data/dataset /data/output/$(basename "$OUTPUT_PATH")"