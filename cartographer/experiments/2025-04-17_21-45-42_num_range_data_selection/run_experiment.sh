#!/bin/bash

# This script must be run inside docker container. 
# It contains the sequence of commands that must be executed to run the experiment.

submap-grid-search \
    --bag_filename /data/bags/2011-01-28-06-37-23 \
    --config_dir /opt/ros/humble/share/cartographer_ros/configuration_files \
    --config_basename mit_stata.lua \
    --rate 1 \
    --samples 30 \
    --output_root /data/maps/num_range_data \
    --grid='{"trajectory_builder.trajectory_builder_2d.submaps.num_range_data": 
                [1, 3, 5, 7, 9, 11, 13, 15, 17, 19, 
                21, 23, 25, 27, 29, 31, 33, 35, 37, 39, 
                41, 43, 45, 47, 49, 51, 53, 55, 57, 59,
                61, 63, 65, 67, 69, 71, 73, 75, 77, 79,
                81, 83, 85, 87, 89, 91, 93, 95, 97, 99]}' \
    --skip_begin 50 \
    --skip_end 60 \
    2>&1 | tee /data/num_accumulated_range_data_select.log