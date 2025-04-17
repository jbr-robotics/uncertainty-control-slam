#!/bin/bash

# This script must be run inside docker container. 
# It contains the sequence of commands that must be executed to run the experiment.

submap-grid-search \
    --bag_filename /data/bags/2011-01-28-06-37-23 \
    --config_dir /opt/ros/humble/share/cartographer_ros/configuration_files \
    --config_basename mit_stata.lua \
    --rate 1 \
    --samples 10 \
    --output_root /data/maps/large_grid \
    --grid='{"trajectory_builder.trajectory_builder_2d.num_accumulated_range_data": [2, 3, 4, 5, 6, 7, 8], 
            "trajectory_builder.trajectory_builder_2d.ceres_scan_matcher.translation_weight": [5, 10, 15, 20, 25, 30, 35], 
            "trajectory_builder.trajectory_builder_2d.ceres_scan_matcher.rotation_weight": [20, 25, 30, 35, 40, 45, 50, 55, 60], 
            "trajectory_builder.trajectory_builder_2d.submaps.num_range_data": [5, 10, 15, 20, 25, 30, 35, 40, 45, 50]}' \
    --skip_begin 50 \
    --skip_end 60 \
    2>&1 | tee /data/num_accumulated_range_data_select.log