Here are 8 most important parameters:

1. `TRAJECTORY_BUILDER_nD.submaps.num_range_data`
Rationale: This parameter is critical for the proper balance between local accuracy and loop closure detection capability. Submaps that are too small or too large can significantly affect the final map quality.

2. `TRAJECTORY_BUILDER_nD.ceres_scan_matcher.translation_weight` and
3. `TRAJECTORY_BUILDER_nD.ceres_scan_matcher.rotation_weight`
Rationale: These weights determine the balance between translational and rotational motion during scan matching, which is especially important when changing the nature of robot movement in different environments.

4. `POSE_GRAPH.fast_correlative_scan_matcher.linear_search_window` and
5. `POSE_GRAPH.fast_correlative_scan_matcher.angular_search_window`
Rationale: These parameters define the search area for the best scan match, which is critical for proper loop closure in various environments. Especially important when there are significant changes in environment scale.

6. `POSE_GRAPH.constraint_builder.loop_closure_translation_weight` and
7. `POSE_GRAPH.constraint_builder.loop_closure_rotation_weight`
Rationale: These parameters directly affect loop closure performance, which is especially important in complex environments with multiple trajectory intersections.

8. `POSE_GRAPH.optimization_problem.global_sampling_ratio`
Rationale: This parameter affects how often global optimization occurs, which is critical for maintaining map consistency under various conditions.

!! Note that these parameters should be tuned sequentially, starting with local SLAM (first 3 parameters) and then moving to global SLAM parameters (remaining parameters).
