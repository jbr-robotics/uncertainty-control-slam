# Cartographer Tuner

Helper package for [Cartographer](https://github.com/ros2/cartographer_ros) tuning.

## Installation
```
pip install cartographer-tuner
```

## PGM Map metrics

The tool implements metrics presented in [2D SLAM Quality Evaluation Methods](https://arxiv.org/abs/1708.02354).

Usage example:

- `pgm-corner-count /path/tp/map.pgm`
- `pgm-occupied-proportion /path/to/map.pgm`
- `pgm-enclosed-areas /path/to/map.pgm`

## Config evaluation methods

TODO

## System Requirements

This package requires the following external tools to be installed (for some functionality):

- `cartographer` - Google Cartographer SLAM library
- `cartographer_ros` - ROS 2 integration for Cartographer
- `rviz` - ROS2 integration for RViz

Dependecy installation:
```
sudo apt-get install \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-rviz
```