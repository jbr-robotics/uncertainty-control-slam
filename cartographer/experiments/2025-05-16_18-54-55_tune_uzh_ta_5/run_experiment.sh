#!/bin/bash

# This script must be run inside docker container. 
# It contains the sequence of commands that must be executed to run the experiment.

submap-grid-search \
    --bag_filename /data/bags/crt_uzh_tracking_area_run2/ \
    --config_dir /opt/ros/humble/share/cartographer_ros/configuration_files \
    --config_basename uzh_tracking_area_run2_2D.lua \
    --rate 1 \
    --samples 10 \
    --output_root /data/maps/uzh_ta_5 \
    --grid='{"trajectory_builder.trajectory_builder_2d.voxel_filter_size": [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.07, 0.08, 0.09, 0.1, 0.11, 0.12, 0.13, 0.14, 0.15, 0.16, 0.17, 0.18, 0.19, 0.2]}' \
    --skip_begin 10 \
    --skip_end 10 \
    2>&1 | tee /data/uzh_ta_5.log