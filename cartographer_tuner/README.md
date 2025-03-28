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

## System Requirements

This package requires the following external tools to be installed (for some functionality):

- `cartographer` - Google Cartographer SLAM library
  - Installation: `sudo apt-get install ros-<version>-cartographer`
- `cartographer_ros` - ROS 2 integration for Cartographer
  - Installation: `sudo apt-get install ros-<version>-cartographer-ros`