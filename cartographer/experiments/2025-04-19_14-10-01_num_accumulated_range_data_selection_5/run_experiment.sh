#!/bin/bash

# This script must be run inside docker container. 
# It contains the sequence of commands that must be executed to run the experiment.

submap-grid-search \
    --bag_filename /data/bags/2011-01-28-06-37-23 \
    --config_dir /opt/ros/humble/share/cartographer_ros/configuration_files \
    --config_basename mit_stata.lua \
    --rate 1 \
    --samples 50 \
    --output_root /data/maps/num_acc_range_data_5 \
    --grid='{"trajectory_builder.trajectory_builder_2d.num_accumulated_range_data": [3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80]}' \
    --skip_begin 50 \
    --skip_end 60 \
    2>&1 | tee /data/num_accumulated_range_data_select.log