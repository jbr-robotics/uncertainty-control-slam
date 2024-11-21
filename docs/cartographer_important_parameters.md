# A list of the most important parameters in cartographer

This document is a list of the most important parameters for 3D SLAM in cartographer along with a brief explanation of what they do.

The list of all parameters can be found here [cartograher_parameters_list.md](/docs/cartographer_parameters_list.md)

> **Note:** 
> One can firstly tune the parameters for local SLAM and then for global SLAM. Thus, it is meaningful to create different lists of important parameters for local and global SLAM.

> **Note:**
> Importance is measured subjectively in terms of map quality, not performance. This assumption allows one to focus on quality firstly and then trade off performance for quality if needed.

> **Note:**
> As mentioned in [this comment](https://github.com/cartographer-project/cartographer_ros/issues/1252#issuecomment-482004124) it is reasonable to firstly tune for 2D SLAM and then for 3D SLAM.

# Important parameters for local SLAM

In author's opinion, these parameters are worth tuning for local SLAM:

- TODO

# Important parameters for global SLAM

In author's opinion, these parameters are worth tuning for global SLAM:

- TODO

# Justification of importance

There is a justification for why listed above parameters are important, while others are not. 

> **Note:** 
> Parameters which never occured in sources are automatically considered unimportant and not described in this document.

## Important parameters 

---

`TRAJECTORY_BUILDER_nD.voxel_filter_size`

- A small cube size will result in a more dense data representation, causing more computations. A large cube size will result in a data loss but will be much quicker. [source](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html#input)

Opinion: highly important to dense scans properly.

---

`high_resolution_adaptive_voxel_filter.max_length`, 
`low_resolution_adaptive_voxel_filter.max_length`,
`high_resolution_adaptive_voxel_filter.min_num_points`,
`low_resolution_adaptive_voxel_filter.min_num_points`

- This filter tries to determine the optimal voxel size (under a max length) to achieve a target number of points. Adaptive voxel filters are used to generate a high resolution and a low resolution point clouds. [source](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html#input)

Opinion: important to dense scans properly.

---

`TRAJECTORY_BUILDER_nD.imu_gravity_time_constant`

- In order to filter the IMU noise, gravity is observed over a certain amount of time. [source](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html#input)

Opinion: for handheld robots it is important to set this value properly to know the acurate gravity vector.

---

`TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_0`,
`TRAJECTORY_BUILDER_3D.ceres_scan_matcher.occupied_space_weight_1`,
`TRAJECTORY_BUILDER_nD.ceres_scan_matcher.translation_weight`,
`TRAJECTORY_BUILDER_nD.ceres_scan_matcher.rotation_weight`

- CeresScanMatcher can be configured to give a certain weight to each of its input.  
[source](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html#local-slam-1)
- the `occupied_space_weight_0` and `occupied_space_weight_1` parameters are related, respectively, to the high resolution and low resolution filtered point clouds.
[source](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html#local-slam-1)
- [This guide](https://web.archive.org/web/20240302164357/https://daily-tech.hatenablog.com/entry/2019/11/25/062304) suggests to start tuning from finding values for these parameters.
- Viam SLAM [mentions](https://github.com/viam-modules/viam-cartographer/blob/main/viam-cartographer/lua_files/mapping_new_map.lua)  these parameters as "tunable".

Opinion: important to prioritize input data.

---

`TRAJECTORY_BUILDER_nD.submaps.num_range_data`

- A submap is considered as complete when the local SLAM has received a given amount of range data. <...>. Submaps must be small enough <...>, so that they are locally correct. On the other hand, they should be large enough to be distinct for loop closure.
[source](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html#local-slam-1)
- In [this guide](https://web.archive.org/web/20240302164357/https://daily-tech.hatenablog.com/entry/2019/11/25/062304) adjusting the value helped to detect issues with loop closure.

Opinion: As described, it is vital to find the best value.

---

`POSE_GRAPH.fast_correlative_scan_matcher.linear_search_window`
`POSE_GRAPH.fast_correlative_scan_matcher_3d.linear_xy_search_window`
`POSE_GRAPH.fast_correlative_scan_matcher_3d.linear_z_search_window`
`POSE_GRAPH.fast_correlative_scan_matcher*.angular_search_window`

- Search window in which the best possible scan alignment will be found.
[source](https://qiita.com/devemin/items/1184d7fc32656b10288e)
- [This comment](https://github.com/cartographer-project/cartographer_ros/issues/1252#issuecomment-483144785) from top-2 cartographer contributor mentions suggests to set these values smaller than 50, 30 metres. 

Opinion: is vital for loop closure.

---

`POSE_GRAPH.constraint_builder.loop_closure_translation_weight`,
`POSE_GRAPH.constraint_builder.loop_closure_rotation_weight`

- Weight used in the optimization problem for the translational/rotational component of loop closure constraints. 
[source](https://qiita.com/devemin/items/1184d7fc32656b10288e)
- Updating loop closure parameters were crutial for making loop closure work in [this guide](https://web.archive.org/web/20240302164357/https://daily-tech.hatenablog.com/entry/2019/11/25/062304)

Opinion: assumed to be vital as it helped to a user to make loop closure work.

---

`POSE_GRAPH.optimization_problem.acceleration_weight`,
`POSE_GRAPH.optimization_problem.rotation_weight`,
`POSE_GRAPH.optimization_problem.local_slam_pose_translation_weight`,
`POSE_GRAPH.optimization_problem.local_slam_pose_rotation_weight`

- Adjusting these parameters helped the author of [this guide](https://web.archive.org/web/20240302164357/https://daily-tech.hatenablog.com/entry/2019/11/25/062304) to make loop closure work.

Opinion: assumed important as it helped to a user to make loop closure work.

---

## Parameters with unknown importance

---

`TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.use_nonmonotonic_steps`,
`TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.max_num_iterations`,
`TRAJECTORY_BUILDER_nD.ceres_scan_matcher.ceres_solver_options.num_threads`

- Ceres optimizes the motion using a descent algorithm for a given number of iterations.
[source](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html#local-slam-1)

Opinion: seems it affects only performance, not the resulting map. However, since ceres solver algorithm is not clear to author, it is diffucult to say if it is important or not.

---

`TRAJECTORY_BUILDER_nD.motion_filter.max_time_seconds`,
`TRAJECTORY_BUILDER_nD.motion_filter.max_distance_meters`,
`TRAJECTORY_BUILDER_nD.motion_filter.max_angle_radians`

- To avoid inserting too many scans per submaps <...> A scan is dropped if the motion that led to it is not considered as significant enough. A scan is inserted into the current submap only if its motion is above a certain distance, angle or time threshold.
[source](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html#local-slam-1)

Opinion: if is not clear if too many scans per submaps is bad for map quality or not.

---

`TRAJECTORY_BUILDER_3D.submaps.high_resolution`,
`TRAJECTORY_BUILDER_3D.submaps.low_resolution`

- Resolution of the ‘high/low_resolution’ map in meters. 
[source](https://qiita.com/devemin/items/1184d7fc32656b10288e)

Opinion: Map resolution is the core element in balancing between map quality and performance. However, it is assumed that it firsly set to some small value (store maps in high resolution) and then, once other parameters are tuned, it can be gradually increased.

---

`POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher.branch_and_bound_depth`,
`POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.branch_and_bound_depth`,
`POSE_GRAPH.constraint_builder.fast_correlative_scan_matcher_3d.full_resolution_depth`
`POSE_GRAPH.constraint_builder.min_score`

- This scan matcher has been specifically designed for Cartographer and makes real-time loop closures scan matching possible. 
[source](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html#global-slam)
- Once the FastCorrelativeScanMatcher has a good enough proposal (above a minimum score of matching) 
[source](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html#global-slam)

Opinion: seems it affect only performance, not the resulting map. However, as fast correlative scan matcher mechanism is not clear to author, it is difficult to say if it is important or not.

---

`POSE_GRAPH.matcher_translation_weight`,
`POSE_GRAPH.matcher_rotation_weight`

- Weight used in the optimization problem for the translational component of non-loop-closure scan matcher constraints.
[source](https://qiita.com/devemin/items/1184d7fc32656b10288e)

Opinion: it mentioned quickly in the [algorithm walkthrough](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html#global-slam) as one of the parameters that worth tuning, but not further explained, so it may be vital for loop closure/matching but effect is not clear.

---

`POSE_GRAPH.optimization_problem.huber_scale`

- Scaling parameter for Huber loss function.
[source](https://qiita.com/devemin/items/1184d7fc32656b10288e)
- Helped to the author of [this guide](https://web.archive.org/web/20240302164357/https://daily-tech.hatenablog.com/entry/2019/11/25/062304) to improve SLAM but did not explained. 
- Mentioned in the [algorithm walkthrough](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html#global-slam) as one of the parameters that worth tuning, however it is not described.

Opinion: importance is not clear as its effect is not clear.

---

## Unimportant parameters

---

`TRAJECTORY_BUILDER_nD.min_range`,
`TRAJECTORY_BUILDER_nD.max_range`,
`TRAJECTORY_BUILDER_3D.missing_data_ray_length`

- Those min and max values should be chosen according to the specifications of your robot and sensors. [source](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html#input)

Opinion: these variables must be set according to the sensor used => no reason to tune them.

---

`POSE_GRAPH.optimize_every_n_nodes = 90` 
- Can be set to 0 to disable global SLAM.  [source](https://google-cartographer-ros.readthedocs.io/en/latest/tuning.html#example-tuning-local-slam)
- How often to execute loop closure. [source](https://qiita.com/devemin/items/1184d7fc32656b10288e)
- In [this guide](https://web.archive.org/web/20240302164357/https://daily-tech.hatenablog.com/entry/2019/11/25/062304) adjusting the parameter did not give any "drastic changes".
- Viam SLAM [mentions](https://github.com/viam-modules/viam-cartographer/blob/main/viam-cartographer/lua_files/mapping_new_map.lua) this parameter as "no reason to change".


Opinion: based on users experience, assumed that the default value is ok.

---

`use_online_correlative_scan_matching = false`, 
`real_time_correlative_scan_matcher.*`

- ... can be enabled if you do not have other sensors or you do not trust them. 
[source](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html#local-slam-1)

Opinion: in current datasets (`HILTI`, `Cartographer examples`) the sensors are accurate enough to not need this feature.

---

`POSE_GRAPH.constraint_builder.sampling_ratio`

- Sampling too few nodes could result in missed constraints and ineffective loop closures. Sampling too many nodes would slow the global SLAM down and prevent real-time loop closures. 
[source](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html#global-slam)

Opinion: assumed it is ok to set the high value and since everything is tuned, one can gradually decrease it if needed.

---

`POSE_GRAPH.max_num_final_iterations`

- This is done to polish the final result of Cartographer and usually does not need to be real-time so a large number of iterations is often a right choice.
[source](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html#global-slam)

Opinion: as described, it is ok to set the high value and since everything is tuned, one can gradually decrease it if needed for real-time.

---

`POSE_GRAPH.constraint_builder.max_constraint_distance`

- Threshold for poses to be considered near a submap.
[source](https://qiita.com/devemin/items/1184d7fc32656b10288e)
- [This comment](https://github.com/cartographer-project/cartographer_ros/issues/1252#issuecomment-483144785) from top-2 cartographer contributor mentions the parameter as "Very uncommon to set". 

Opinion: seems to be vital for loop closure, however highlighted as unimportant by cartographer contributors.

---

`POSE_GRAPH.optimization_problem.odometry_translation_weight`,
`POSE_GRAPH.optimization_problem.odometry_rotation_weight`

- Helped to the author of [this guide](https://web.archive.org/web/20240302164357/https://daily-tech.hatenablog.com/entry/2019/11/25/062304) to make loop closure work.

Opinion: despite effect given in the guide, it seems it is not important as odometry information is not provided in the datasets used: `HILTI`, `Cartographer examples`.

---

`TRAJECTORY_BUILDER_nD.num_accumulated_range_data = 1`

- How many scans to accumulate before doing scan matching. 
[source1](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html), 
[source2](https://qiita.com/devemin/items/1184d7fc32656b10288e)
- On RPidar only value of 1 is ok
[source](https://medium.com/@kabilankb2003/ros2-humble-cartographer-on-nvidia-jetson-nano-with-rplidar-c0dea4480b78)

Opinion: based on example, assumed that setting value to 1 is ok.

---


# References
- [Cartographer Official Tuning Guide](https://google-cartographer-ros.readthedocs.io/en/latest/tuning.html#)
- [Cartographer Official Algorithm Walkthrough for tuning](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html)
- [Cartographer Tuning Guide by Sheaffer](https://sheaffej.github.io/b2/slam/tuning-cartographer.html)
- [Cartographer Tuning Discussion](https://github.com/cartographer-project/cartographer_ros/issues/1252)
- [Cartographer Tuning Process Example](https://web.archive.org/web/20240302164357/https://daily-tech.hatenablog.com/entry/2019/11/25/062304)
- [Viam Cartographer guide](https://docs.viam.com/services/slam/cartographer/)
- [Viam Cartographer Default Parameters](https://github.com/viam-modules/viam-cartographer/blob/main/viam-cartographer/lua_files/mapping_new_map.lua)
- [Cartographer Tuning Guide on Medium](https://medium.com/@kabilankb2003/ros2-humble-cartographer-on-nvidia-jetson-nano-with-rplidar-c0dea4480b78)
- [Ouster Cartographer Example (with configs provided)](https://github.com/Krishtof-Korda/ouster_example_cartographer)
- [Community cartographer parameters documentation](https://qiita.com/devemin/items/1184d7fc32656b10288e)
