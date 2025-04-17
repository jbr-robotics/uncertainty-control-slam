#!/bin/bash

# This script must be run inside docker container. 
# It contains the sequence of commands that must be executed to run the experiment.

submap-grid-search \
    --bag_filename /data/bags/2011-01-28-06-37-23 \
    --config_dir /opt/ros/humble/share/cartographer_ros/configuration_files \
    --config_basename mit_stata.lua \
    --rate 1 \
    --samples 30 \
    --output_root /data/maps/num_acc_range_data_4 \
    --grid='{"trajectory_builder.trajectory_builder_2d.num_accumulated_range_data": [5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125, 130, 135, 140, 145, 150]}' \
    --skip_begin 50 \
    --skip_end 60 \
    2>&1 | tee /data/num_accumulated_range_data_select.log