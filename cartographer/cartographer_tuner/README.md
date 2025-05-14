# Cartographer Tuner

Helper package for [Cartographer](https://github.com/ros2/cartographer_ros) tuning and optimization.

## Overview

The Cartographer Tuner provides tools to evaluate and optimize Cartographer SLAM configurations automatically. It implements various metrics to evaluate map quality and provides optimization methods to find the best parameter values for specific datasets.

## Installation
```
pip install cartographer-tuner
```

### System Requirements

This package requires the following external tools to be installed (for some functionality):

- `cartographer` - Google Cartographer SLAM library
- `cartographer_ros` - ROS 2 integration for Cartographer
- `rviz` - ROS2 visualization tool

**Dependency installation:**
```
sudo apt-get install \
    ros-${ROS_DISTRO}-cartographer-ros \
    ros-${ROS_DISTRO}-cartographer \
    ros-${ROS_DISTRO}-cartographer-rviz
```

## Map Quality Metrics

The tool implements several map quality metrics based on the paper [2D SLAM Quality Evaluation Methods](https://arxiv.org/abs/1708.02354):

### Available Metrics

1. **Corner Count** - Measures the number of corners in a map (higher values typically indicate more detailed maps)
   ```
   pgm-corner-count /path/tp/map.pgm
   ```

2. **Occupied Proportion** - Calculates the proportion of occupied cells in the map
   ```
   pgm-occupied-proportion /path/to/map.pgm
   ```

3. **Enclosed Areas** - Counts the number of enclosed areas in the map
   ```
   pgm-enclosed-areas /path/to/map.pgm
   ```

4. **Uncertainty Proportion** - Measures the proportion of cells that are classified as "uncertain"
   ```
   pgm-uncertainty-proportion /path/to/map.pgm
   ```
   
   This metric uses K-Means clustering (K=3) to dynamically classify cells into:
   - Occupied (lowest intensity cluster)
   - Unoccupied (highest intensity cluster)
   - Uncertain (middle cluster)
   
   The proportion is calculated as: `uncertain_cells / total_known_cells`

## Map Building 

The tool can build 2D maps from rosbag files using Cartographer. 

> **Note**: rosbag files must contain topics named according to Cartographer requirements: `/imu`, `/scan`, etc.

Example:
```
lua-to-pgm \
    --bag_filename=/data/bags/2011-01-28-06-37-23 \
    --configuration_directory=/opt/ros/humble/share/cartographer_ros/configuration_files \
    --configuration_basenames=mit_stata.lua \
    --map_filestem=/data/maps/2011-01-28-06-37-23_test 
```

## Configuration Evaluation

You can combine map building and evaluation in one step to assess the quality of a specific configuration:

```
lua-pgm-metrics \
    --bag_filename=/data/bags/2011-01-28-06-37-23 \
    --config_dir=/opt/ros/humble/share/cartographer_ros/configuration_files \
    --config_basename=mit_stata.lua
```

## Configuration Optimization

The Cartographer Tuner provides several optimization strategies to find optimal parameter values based on the quality metrics:

### Grid Search

Performs an exhaustive search over specified parameter ranges:

```
config-grid-search \
    --bag_filename=/data/bags/2011-01-28-06-37-23 \
    --config_dir=/opt/ros/humble/share/cartographer_ros/configuration_files \
    --config_basename=mit_stata.lua \
    --grid '{"map_builder.pose_graph.constraint_builder.ceres_scan_matcher.translation_weight": [0.1, 1, 10, 100], "map_builder.pose_graph.constraint_builder.ceres_scan_matcher.rotation_weight": [0.1, 1, 10, 100]}' \
    --metrics corner_count,enclosed_areas_count,occupied_proportion \
    --output=/data/ceres_weights_metrics.csv
```

### HyperOpt Search

Leverages the HyperOpt library for more sophisticated parameter optimization using Bayesian techniques:

```
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
    --config_basename mit_stata.lua \
    --bag_filename /data/bags/2011-01-28-06-37-23 \
    --output_root /data/maps/experiment_output \
    --max_evals 300 \
    --samples 5
```

Key features of HyperOpt Search:
- **Advanced Search Algorithms**: Uses Tree-structured Parzen Estimator (TPE) for intelligent parameter exploration
- **Continuous Parameter Space**: Can handle continuous ranges with various distributions (uniform, log-uniform)
- **Efficient Optimization**: Focuses sampling on promising regions of the parameter space
- **Trial History**: Maintains history of all trials for post-analysis

The tool automatically optimizes parameters to minimize the uncertainty metric, helping to find configurations that produce more certain maps.

### How to Choose Parameters for Optimization

Common parameters that benefit from optimization include:

1. **Translation and Rotation Weights**:
   - `map_builder.pose_graph.constraint_builder.ceres_scan_matcher.translation_weight`
   - `map_builder.pose_graph.constraint_builder.ceres_scan_matcher.rotation_weight`
   - `trajectory_builder.trajectory_builder_2d.ceres_scan_matcher.translation_weight`
   - `trajectory_builder.trajectory_builder_2d.ceres_scan_matcher.rotation_weight`

2. **Constraint Builder Parameters**:
   - `map_builder.pose_graph.constraint_builder.sampling_ratio`
   - `map_builder.pose_graph.constraint_builder.min_score`
   - `map_builder.pose_graph.constraint_builder.global_localization_min_score`

3. **Global Optimization Parameters**:
   - `map_builder.pose_graph.optimization_problem.huber_scale`
   - `map_builder.pose_graph.optimization_problem.odometry_translation_weight`
   - `map_builder.pose_graph.optimization_problem.odometry_rotation_weight`

## Analyzing Results

The optimization process generates CSV files with metric values for each parameter combination. These can be analyzed to determine which parameter values produce the best results for your specific dataset.

Example analysis (using tools like pandas):

```python
import pandas as pd
import matplotlib.pyplot as plt

results = pd.read_csv('/data/ceres_weights_metrics.csv')
best_config = results.sort_values('corner_count', ascending=False).iloc[0]
print(f"Best parameters: {best_config}")

# Plot results
plt.figure(figsize=(10, 6))
plt.scatter(results['translation_weight'], results['corner_count'])
plt.xlabel('Translation Weight')
plt.ylabel('Corner Count')
plt.title('Impact of Translation Weight on Map Quality')
plt.savefig('translation_weight_analysis.png')
```

## Experimental Results

Check the `experiments` directory for examples of optimization runs with different parameters and datasets. Each experiment contains:

- Configuration files
- Run scripts
- Result visualizations
- Analysis notes

These experiments demonstrate how different parameter settings affect map quality metrics and can guide your own parameter tuning process.
