# Manual Parameters

This file contains the list of parameters that must or should be tuned manually rather than be optimized during tuning.

- `TRAJECTORY_BUILDER_nD.min_range`, `TRAJECTORY_BUILDER_nD.max_range`: must be selected [according to the specification of the robot and sensors](https://google-cartographer-ros.readthedocs.io/en/latest/algo_walkthrough.html#input).


# Tunable parameters

This document suggests the list of Cartographer Parameters, and their min, max and default values (taken from default [configs](https://github.com/cartographer-project/cartographer/tree/df337194e21f98f8c7b0b88dab33f878066d4b56/configuration_files)) with rationale of the decision for such min max range. This range then is suggested to be used in tinung. Thus, there are listed only parameters which are subject for optimization.

- `TRAJECTORY_BUILDER_2D.ceres_scan_matcher.occupied_space_weight = 1`, `TRAJECTORY_BUILDER_3D.ceres_scan_matcher.translation_weight = 10`, `TRAJECTORY_BUILDER_3D.ceres_scan_matcher.rotation_weight = 40`. 
Range: $[1, 100000]$ (because too large weights are unreasonable)

# Untouched parameters
Below are listed parameters for which the intition says to leave them at default values and do not tune them. 

- `TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025`: assumed that changing it makes sense only in very specific environments or for speeding up SLAM. 

# Unknown parameters

Below are listed parameters, for which there is no intuition how to tune them

- `TRAJECTORY_BUILDER_2D.missing_data_ray_length`
- `TRAJECTORY_BUILDER_nD.*adaptive_voxel_filter.max_length`
- `TRAJECTORY_BUILDER_nD.*adaptive_voxel_filter.min_num_points`
