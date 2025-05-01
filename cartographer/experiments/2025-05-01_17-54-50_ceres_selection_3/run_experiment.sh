#!/bin/bash

# This script must be run inside docker container. 
# It contains the sequence of commands that must be executed to run the experiment.

submap-hyperopt-search \
    --search_space_def='{
        "trajectory_builder.trajectory_builder_2d.ceres_scan_matcher.translation_weight": {
            "type": "float",
            "min": 0.001,
            "max": 10000
        },
        "trajectory_builder.trajectory_builder_2d.ceres_scan_matcher.rotation_weight": {
            "type": "float",
            "min": 0.001,
            "max": 10000
        }
    }' \
    --config_dir /opt/ros/humble/share/cartographer_ros/configuration_files \
    --config_basename mit_stata.lua \
    --bag_filename /data/bags/2011-01-28-06-37-23 \
    --output_root /data/maps/ceres_3 \
    --max_evals 100 \
    --samples 10 \
    --rate 1 \
    --skip_begin 50 \
    --skip_end 60 \
    2>&1 | tee /data/maps/ceres_3/hyperopt_tuning.log