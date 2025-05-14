# Cartographer

This directory contains a Docker environment for Cartographer.

## Build and Run Docker

### Build Docker
```sh
./build_docker.sh
```

### Run Docker
```sh
./run_docker.sh
```

> **Note:** The script creates the `cartographer/data` directory and mounts it to `/data`, allowing it to be used for data sharing.

## Quick Start

Refer to [EXAMPLE.md](./EXAMPLE.md) for step-by-step examples on getting started with Cartographer.

## Additional Tools Available Inside the Container

### `rosbags-convert`

To convert a `rosbag` file into the `rosbag2` format, use the following command:

```sh
rosbags-convert --src /path/to/dataset.bag --dst /path/to/dataset
```

### `ros2 bag rename`

Since Cartographer expects rosbag files to have specific topic names, and not all tools in this repository support topics remapping, it is recommended to use the `ros2bag_tools` to rename the topics as specified in the [documentation](https://google-cartographer-ros.readthedocs.io/en/latest/ros_api.html#subscribed-topics).

```sh
ros2 bag rename \
    -o /path/to/renamed_bag_dir \
    -t /original_topic_name --name /new_topic_name \
    -t /original_topic_name2 --name /new_topic_name2 \
    /path/to/bag_dir/
```

### Parameter Optimizers

This repository includes two parameter optimization tools to improve SLAM performance for specific datasets:

#### 1. Grid Search Optimizer

##### How It Works

The parameter optimizer systematically tests different combinations of Cartographer parameter values and evaluates the resulting map quality. This process helps identify optimal parameter settings that produce the best SLAM results for your specific use case.

##### Key Features:

- **Grid Search Optimization:** Tests all combinations of specified parameter values
- **Multiple Quality Metrics:** Evaluates maps using various metrics (corner count, enclosed areas, occupied space)
- **CSV Output:** Generates structured results for further analysis

##### Example Usage:

```sh
cd /root

python3 -m quality_estimator.parameter_optimizer \
    --bag_filename=/data/kitti_2011_09_26_drive_0117_synced \
    --config_dir=/opt/ros/humble/share/cartographer_ros/configuration_files \
    --config_basename=kitti_3D.lua \
    --points2_topic=/kitti/velo/pointcloud \
    --imu_topic=/kitti/oxts/imu \
    --skip_seconds=0 \
    --parameter_grid '{"map_builder.pose_graph.constraint_builder.ceres_scan_matcher_2d.translation_weight": [0.01, 0.1, 1, 10, 100], "map_builder.pose_graph.constraint_builder.ceres_scan_matcher_2d.rotation_weight": [0.01, 0.1, 1, 10, 100]}' \
    --csv_output=/data/kitti_ceres_weights.csv \
    | tee /data/find_kitti_ceres_weights.log
```

#### 2. HyperOpt-based Optimizer

This advanced optimizer uses the HyperOpt library for Bayesian optimization of parameters, offering more sophisticated parameter search strategies.

##### Key Features:

- **Bayesian Optimization:** Uses Tree-structured Parzen Estimator (TPE) algorithm to efficiently search parameter space
- **Continuous Parameter Ranges:** Can explore continuous parameter ranges with various distributions (uniform, log-uniform)
- **Uncertainty Metric:** Optimizes based on map uncertainty proportion using K-means clustering to identify uncertain areas
- **Submap Analysis:** Evaluates multiple submaps for more robust parameter assessment

##### Example Usage:

```sh
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
    --config_dir=/opt/ros/humble/share/cartographer_ros/configuration_files \
    --config_basename=mit_stata.lua \
    --bag_filename=/data/bags/2011-01-28-06-37-23 \
    --output_root=/data/maps/experiment_output \
    --max_evals=300 \
    --samples=5
```

##### Command Parameters:

- `--search_space_def`: JSON definition of parameters to optimize with their ranges and distributions
- `--config_dir`: Directory containing Cartographer configuration files
- `--config_basename`: Base filename of the configuration to use
- `--bag_filename`: Path to the ROS bag file
- `--output_root`: Directory to store optimization results and intermediate files
- `--max_evals`: Maximum number of parameter combinations to evaluate
- `--samples`: Number of submaps to generate for each parameter set

For more advanced optimization capabilities, check out the [Cartographer Tuner](./cartographer_tuner) module, which provides additional metrics and optimization strategies.

### Experiments

The `experiments` directory contains examples of different optimization runs with analysis results. These can serve as reference for your own parameter tuning efforts.

## Additional Notes

### Map Visualization

- To visualize `.ply` files, use [MeshLab](https://www.meshlab.net/#download) or [point_cloud_viewer](/point_cloud_viewer/).

### Map Quality Estimation

> **Recommendation:** Use the [parameter optimizers](#parameter-optimizers) for fine-tuning instead of relying solely on the information in this section.

> **Note:** As mentioned in the [Cartographer documentation](https://google-cartographer.readthedocs.io/en/latest/evaluation.html#advantages-limitations), this estimation method is valid only for local SLAM evaluations with optimizations disabled.

#### Steps:

1. Generate an optimized map as described in the [EXAMPLE.md](./EXAMPLE.md). This will create a map with optimized relations. Assume the generated file is named `uzh_tracking_area_run2_map.pbstream`.
 
2. Auto-generate ground truth using:
   ```sh
   cartographer_autogenerate_ground_truth \
       -pose_graph_filename /path/to/uzh_tracking_area_run2_map.pbstream \
       -output_filename /path/to/uzh_tracking_area_run2_relations.pbstream \
       -min_covered_distance 20 \
       -outlier_threshold_meters 0.15 \
       -outlier_threshold_radians 0.02
   ```
   This retains only relations that meet the following thresholds:
   - `min_covered_distance`: Loop closures before reaching this distance are not considered.
   - `outlier_threshold_meters`, `outlier_threshold_radians`: Constraints beyond these distance/angle thresholds are excluded.

3. Disable optimization by setting `POSE_GRAPH.optimize_every_n_nodes = 0` in the `.lua` configuration file, then repeat the previous steps. Assume the generated file is named `uzh_tracking_area_run2_test.pbstream`.

4. You should now have three files:
   - `uzh_tracking_area_run2_map.pbstream` (test map)
   - `uzh_tracking_area_run2_relations.pbstream` (relations file)
   - `uzh_tracking_area_run2_test.pbstream` (unoptimized map)

5. To estimate local SLAM quality, run:
   ```sh
   cartographer_compute_relations_metrics \
       -relations_filename /path/to/uzh_tracking_area_run2_relations.pbstream \
       -pose_graph_filename /path/to/uzh_tracking_area_run2_test.pbstream
   ```

#### Example Output:

```sh
Abs translational error 0.26454 +/- 0.13541 m
Sqr translational error 0.08812 +/- 0.08872 m^2
Abs rotational error 1.80877 +/- 1.01483 deg
Sqr rotational error 4.29034 +/- 4.92501 deg^2
```
