#!/bin/bash

# This script must be run inside docker container. 
# It contains the sequence of commands that must be executed to run the experiment.

submap-grid-search \
    --bag_filename /data/bags/crt_uzh_tracking_area_run2/ \
    --config_dir /opt/ros/humble/share/cartographer_ros/configuration_files \
    --config_basename uzh_tracking_area_run2_2D.lua \
    --rate 1 \
    --samples 20 \
    --output_root /data/maps/uzh_ta_1 \
    --grid='{"trajectory_builder.trajectory_builder_2d.num_accumulated_range_data": [5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120]}' \
    --skip_begin 10 \
    --skip_end 10 \
    2>&1 | tee /data/uzh_ta_1.log