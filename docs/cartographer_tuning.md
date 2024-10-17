This page is based on the [doc](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html)


![Relations](https://raw.githubusercontent.com/cartographer-project/cartographer/master/docs/source/high_level_system_overview.png)


Two major parts:
- **Local SLAM**
	**Job:** build sub-maps
	[Example 2D config](https://github.com/cartographer-project/cartographer/blob/df337194e21f98f8c7b0b88dab33f878066d4b56/configuration_files/trajectory_builder_2d.lua)
	[Exampe 3D config](https://github.com/cartographer-project/cartographer/blob/df337194e21f98f8c7b0b88dab33f878066d4b56/configuration_files/trajectory_builder_3d.lua)
- **Global SLAM**
	**Job:** find loop closure constraints
	[Example config](https://github.com/cartographer-project/cartographer/blob/df337194e21f98f8c7b0b88dab33f878066d4b56/configuration_files/pose_graph.lua)

## LIDAR related parameters
Parameters to cut out out-of-range measurements. Depends on `LIDAR` hardware.
```
TRAJECTORY_BUILDER_nD.min_range
TRAJECTORY_BUILDER_nD.max_range
```

Parameters to convert 3D scans into 2D
```
TRAJECTORY_BUILDER_2D.min_z = -1. 
TRAJECTORY_BUILDER_2D.max_z = 1. 
```

The length of a scan batch sent to Cartographer with some frequency. 
```
TRAJECTORY_BUILDER_nD.num_accumulated_range_data
```

To create a denser point cloud, it is split into voxels of a given size, and only the centroid is presented for each voxel.
```
TRAJECTORY_BUILDER_nD.voxel_filter_size
```

The same could be done via adaptive voxel sizes: 
```
TRAJECTORY_BUILDER_nD.*adaptive_voxel_filter.max_length
TRAJECTORY_BUILDER_nD.*adaptive_voxel_filter.min_num_points

TRAJECTORY_BUILDER_nD.low_resolution_adaptive_voxel_filter.max_length
TRAJECTORY_BUILDER_nD.low_resolution_adaptive_voxel_filter.min_num_points
```

# IMU related parameters

```
TRAJECTORY_BUILDER_2D.use_imu_data
TRAJECTORY_BUILDER_nD.imu_gravity_time_constant
```

# Local SLAM 
2 Strategies available:

## `CeresScanMatcher` 
fast but cannot fix significant errors

Importance of the inputs can be adjusted by specifying dimensionless quantities
```
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1e1
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0
TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1
TRAJECTORY_BUILDER_nD.ceres_scan_matcher.translation_weight = 1e-2
TRAJECTORY_BUILDER_nD.ceres_scan_matcher.rotation_weight = 1e-2
```

The values above are suggested based on [this issue](https://github.com/cartographer-project/cartographer_ros/issues/1353)

Other parameters
```
TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps
TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.max_num_iterations
TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.num_threads
```

## `RealTimeCorrelativeScanMatcher`
expensive but robust in feature expensive environments
**TBD.**
Parameters
```
TRAJECTORY_BUILDER_nD.use_online_correlative_scan_matching
TRAJECTORY_BUILDER_nD.real_time_correlative_scan_matcher.linear_search_window
TRAJECTORY_BUILDER_nD.real_time_correlative_scan_matcher.angular_search_window
TRAJECTORY_BUILDER_nD.real_time_correlative_scan_matcher.translation_delta_cost_weight
TRAJECTORY_BUILDER_nD.real_time_correlative_scan_matcher.rotation_delta_cost_weight
```

## Motion filter
Drops scans if they are not significant enough
```
TRAJECTORY_BUILDER_nD.motion_filter.max_time_seconds
TRAJECTORY_BUILDER_nD.motion_filter.max_distance_meters
TRAJECTORY_BUILDER_nD.motion_filter.max_angle_radians
```

## Sub-Maps
Number of ranges to be received before sub-map is complete
```
TRAJECTORY_BUILDER_nD.submaps.num_range_data
```

2D grid type
```
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.grid_type
```

Range data insertion parameters
```
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.hit_probability
TRAJECTORY_BUILDER_2D.submaps.range_data_inserter.probability_grid_range_data_inserter.miss_probability

TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.hit_probability
TRAJECTORY_BUILDER_3D.submaps.range_data_inserter.miss_probability
```

More parameters
```
TRAJECTORY_BUILDER_2D.submaps.grid_options_2d.resolution
TRAJECTORY_BUILDER_3D.submaps.high_resolution
TRAJECTORY_BUILDER_3D.submaps.low_resolution
TRAJECTORY_BUILDER_3D.high_resolution_adaptive_voxel_filter.max_range
TRAJECTORY_BUILDER_3D.low_resolution_adaptive_voxel_filter.max_range
```

# Global SLAM

Optimize global graph once a certain number of trajectory nodes was inserted.
```
POSE_GRAPH.optimize_every_n_nodes
```
If set to 0, then global SLAM is disables

## Constraints parameters
```
POSE_GRAPH.constraint_builder.max_constraint_distance
POSE_GRAPH.fast_correlative_scan_matcher.linear_search_window
POSE_GRAPH.fast_correlative_scan_matcher_3d.linear_xy_search_window
POSE_GRAPH.fast_correlative_scan_matcher_3d.linear_z_search_window
POSE_GRAPH.fast_correlative_scan_matcher*.angular_search_window
```

Rate of nodes to sub-sample to consider them for constraint building
```
POSE_GRAPH.constraint_builder.sampling_ratio
```

More parameters

<span style="color: red">TODO: describe them in more detail</span>

```
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth
POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.full_resolution_depth

POSE_GRAPH.constraint_builder.min_score
POSE_GRAPH.constraint_builder.ceres_scan_matcher_3d
POSE_GRAPH.constraint_builder.ceres_scan_matcher

POSE_GRAPH.constraint_builder.loop_closure_translation_weight
POSE_GRAPH.constraint_builder.loop_closure_rotation_weight
POSE_GRAPH.matcher_translation_weight
POSE_GRAPH.matcher_rotation_weight
POSE_GRAPH.optimization_problem.*_weight
POSE_GRAPH.optimization_problem.ceres_solver_options

POSE_GRAPH.optimization_problem.log_solver_summary
POSE_GRAPH.optimization_problem.use_online_imu_extrinsics_in_3d

POSE_GRAPH.optimization_problem.huber_scale

POSE_GRAPH.max_num_final_iterations
```

