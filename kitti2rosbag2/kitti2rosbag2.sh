#!/bin/bash

# Validate the number of arguments
if [ "$#" -ne 4 ]; then
    echo "Usage: $0 <sequence_no> <dataset_dir> <odometry_dir> <output_bag_dir>"
    exit 1
fi

# Capture the arguments
SEQUENCE_NO=$1
DATASET_DIR=$2
ODOMETRY_DIR=$3
OUTPUT_BAG_DIR=$4

# Run the Docker container and execute the launch file
docker run --rm --entrypoint "" \
    -v "$DATASET_DIR":/data/dataset:ro \
    -v "$ODOMETRY_DIR":/data/odometry:ro \
    -v "$OUTPUT_BAG_DIR":/data/output \
    kitti2rosbag2:latest \
    bash -c "source /opt/ros/humble/setup.bash && \
            source /root/ros2_ws/install/setup.bash && \
            ros2 launch kitti2rosbag2 kitti2rosbag2.launch.py \
            sequence:=$SEQUENCE_NO \
            data_dir:=/data/dataset \
            odom_dir:=/data/odometry \
            bag_dir:=/data/output"
