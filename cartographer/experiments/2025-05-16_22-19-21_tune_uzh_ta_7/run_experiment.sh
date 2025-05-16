#!/bin/bash

# This script must be run inside docker container. 
# It contains the sequence of commands that must be executed to run the experiment.

submap-hyperopt-search \
    --search_space_def='{
        "trajectory_builder.trajectory_builder_2d.ceres_scan_matcher.translation_weight": {
            "type": "float",
            "min": 0.001,
            "max": 10000,
            "distribution": "loguniform"
        },
        "trajectory_builder.trajectory_builder_2d.ceres_scan_matcher.rotation_weight": {
            "type": "float",
            "min": 0.001,
            "max": 10000,
            "distribution": "loguniform"
        }
    }' \
    --config_dir /opt/ros/humble/share/cartographer_ros/configuration_files \
    --config_basename uzh_tracking_area_run2_2D.lua \
    --bag_filename /data/bags/crt_uzh_tracking_area_run2/ \
    --output_root /data/maps/uzh_ta_7 \
    --max_evals 150 \
    --samples 5 \
    --rate 1 \ 
    --skip_begin 10 \
    --skip_end 10 \
    2>&1 | tee /data/uzh_ta_7.log