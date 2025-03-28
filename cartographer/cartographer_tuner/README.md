# Cartographer Tuner

Helper package for [Cartographer](https://github.com/ros2/cartographer_ros) tuning.

## Installation
```
pip install cartographer-tuner
```

### System Requirements

This package requires the following external tools to be installed (for some functionality):

- `cartographer` - Google Cartographer SLAM library
- `cartographer_ros` - ROS 2 integration for Cartographer
- `rviz` - ROS2 integration for RViz

**Dependecy installation:**
```
sudo apt-get install \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-rviz
```

## PGM Map metrics

The tool implements metrics presented in [2D SLAM Quality Evaluation Methods](https://arxiv.org/abs/1708.02354).

Example

- `pgm-corner-count /path/tp/map.pgm`
- `pgm-occupied-proportion /path/to/map.pgm`
- `pgm-enclosed-areas /path/to/map.pgm`

## Map building 

The tool may build 2D map out of rosbag file via cartographer. 

_Note_: rosbag file must contain topics named according to cartographer requirements: `/imu`, `/scan`, etc

Example:
```
lua-to-pgm \
    --bag_filename=/data/bags/2011-01-28-06-37-23 \
    --configuration_directory=/opt/ros/humble/share/cartographer_ros/configuration_files \
    --configuration_basenames=mit_stata.lua \
    --map_filestem=/data/maps/2011-01-28-06-37-23_test 
```

## Configuration evaluation

One can combine map building and evaluation in one step.

Example:
```
lua-pgm-metrics \
    --bag_filename=/data/bags/2011-01-28-06-37-23 \
    --config_dir=/opt/ros/humble/share/cartographer_ros/configuration_files \
    --config_basename=mit_stata.lua
```

## Configuration optimization

One can optimize their configuration using metrics described above and one of available optimizators:

- Grid Search: 
    ```
    config-grid-search \
        --bag_filename=/data/bags/2011-01-28-06-37-23 \
        --config_dir=/opt/ros/humble/share/cartographer_ros/configuration_files \
        --config_basename=mit_stata.lua \
        --grid '{"map_builder.pose_graph.constraint_builder.ceres_scan_matcher_3d.translation_weight": [0.1, 1, 10, 100], "map_builder.pose_graph.constraint_builder.ceres_scan_matcher_3d.rotation_weight": [0.1, 1, 10, 100]}' \
        --metrics corner_count,enclosed_areas_count,occupied_proportion \
        --output=/data/ceres_weights_metrics.csv
    ```
