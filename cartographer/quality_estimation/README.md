# How to use

example luanch command:
```
python3 -m quality_estimator \
    --bag_filename=/workspace/bags/uzh_tracking_area_run2 \
    --config_dir=/opt/ros/humble/share/cartographer_ros/configuration_files/ \
    --config_basename=uzh_tracking_area_run2_3D.lua \
    --points2_topic=/os_cloud_node/points \
    --imu_topic=/os_cloud_node/imu \
    --skip_seconds=60
```

```
python3 -m quality_estimator.parameter_optimizer \
    --bag_filename=/workspace/bags/uzh_tracking_area_run2 \
    --config_dir=/opt/ros/humble/share/cartographer_ros/configuration_files \
    --config_basename=uzh_tracking_area_run2_3D.lua \
    --points2_topic=/os_cloud_node/points \
    --imu_topic=/os_cloud_node/imu \
    --parameter_grid '{"trajectory_builder.trajectory_builder_3d.ceres_scan_matcher.translation_weight": [15, 20], "map_builder.pose_graph.optimize_every_n_nodes": [30, 60]}' \
    --skip_seconds=60 \
    | tee log.log
```

# Trobleshooting

It is recommended to firstly lauch each steps manually, to verify that everything is ok on each step.

---

If there are too many such messages and almost no constraints
```
0 computations resulted in 0 additional constraints.
```
, then one may need to decrease to make sure that constraints building is working:
```
POSE_GRAPH.constraint_builder.min_score
POSE_GRAPH.constraint_builder.global_localization_min_score 

```

---

Make sure that 
`python3 autogenerate_ground_truth_launcher.py` with its args Writes more than 0 relations 
Something similar is expected in the log:
```
I1219 19:49:43.672624 29956 autogenerate_ground_truth_main.cc:61] Writing 170 relations to '/workspace/bags/uzh_ground_truth.pbstream'.
```

If 0 relations are written, one may consider adjusting parameters:
```
-min_covered_distance # decrease 
-outlier_threshold_meters # increase
-outlier_threshold_radians # increase
```

---
