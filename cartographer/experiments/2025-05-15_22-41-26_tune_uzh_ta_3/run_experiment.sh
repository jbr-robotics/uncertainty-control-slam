#!/bin/bash

# This script must be run inside docker container. 
# It contains the sequence of commands that must be executed to run the experiment.

submap-grid-search \
    --bag_filename /data/bags/crt_uzh_tracking_area_run2/ \
    --config_dir /opt/ros/humble/share/cartographer_ros/configuration_files \
    --config_basename uzh_tracking_area_run2_2D.lua \
    --rate 1 \
    --samples 20 \
    --output_root /data/maps/uzh_ta_3 \
    --grid='{"trajectory_builder.trajectory_builder_2d.min_z": [-2, -1.5, -1, -0.5, 0.1],
    "trajectory_builder.trajectory_builder_2d.max_z": [0.1, 0.5, 1, 1.5, 2]}' \
    --skip_begin 10 \
    --skip_end 10 \
    2>&1 | tee /data/uzh_ta_3.log