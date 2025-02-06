# Cartographer

This directory contains a Docker environment for Cartographer.

## Build and Run Docker

### Build Docker
```sh
./build_docker.sh
```

### Run Docker
```sh
./run_cartographer_docker.sh
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

### Parameter Optimizer

This tool optimizes Cartographer parameters using a grid search and integrates the [Cartographer evaluation tool](https://google-cartographer.readthedocs.io/en/latest/evaluation.html) for quality estimation.

#### Example Usage:

```sh
cd /root

python3 -m quality_estimator.parameter_optimizer \
    --bag_filename=/data/kitti_2011_09_26_drive_0117_synced \
    --config_dir=/opt/ros/humble/share/cartographer_ros/configuration_files \
    --config_basename=kitti_3D.lua \
    --points2_topic=/kitti/velo/pointcloud \
    --imu_topic=/kitti/oxts/imu \
    --skip_seconds=0 \
    --parameter_grid '{"map_builder.pose_graph.constraint_builder.ceres_scan_matcher_3d.translation_weight": [0.01, 0.1, 1, 10, 100], "map_builder.pose_graph.constraint_builder.ceres_scan_matcher_3d.rotation_weight": [0.01, 0.1, 1, 10, 100]}' \
    --csv_output=/data/kitti_ceres_weights.csv \
    | tee /data/find_kitti_ceres_weights.log
```

## Additional Notes

### Map Visualization

- To visualize `.ply` files, use [MeshLab](https://www.meshlab.net/#download) or [point_cloud_viewer](/point_cloud_viewer/).

### Map Quality Estimation

> **Recommendation:** Use the [parameter optimizer](#parameter-optimizer) for fine-tuning instead of relying solely on the information in this section.

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
