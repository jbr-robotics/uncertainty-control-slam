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
    --config_basename kitti_2D.lua \
    --bag_filename /data/bags/crt_kitti_2011_09_26_drive_0117_synced/ \
    --output_root /data/maps/kitti_1 \
    --max_evals 50 \
    --samples 1 \
    --rate 1 \
    --skip_begin 1 \
    --skip_end 1 \
    2>&1 | tee /data/kitti_1.log