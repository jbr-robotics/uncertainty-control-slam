#!/bin/bash

# This script must be run inside docker container. 
# It contains the sequence of commands that must be executed to run the experiment.

submap-grid-search \
    --bag_filename /data/bags/2011-01-28-06-37-23 \
    --config_dir /opt/ros/humble/share/cartographer_ros/configuration_files \
    --config_basename mit_stata.lua \
    --rate 1 \
    --samples 10 \
    --output_root /data/maps/ceres_2 \
    --grid='{ 
            "trajectory_builder.trajectory_builder_2d.ceres_scan_matcher.translation_weight": [0.001, 0.01, 0.1, 1, 10, 100, 1000, 10000], 
            "trajectory_builder.trajectory_builder_2d.ceres_scan_matcher.rotation_weight": [0.001, 0.01, 0.1, 1, 10, 100, 1000, 10000] 
            }' \
    --skip_begin 50 \
    --skip_end 60 \
    2>&1 | tee /data/num_accumulated_range_data_select.log