# Cartographer example

This directory contains files and guides on running Cartographer

# Preparation

1. Build docker `docker build -t ros_cart .`
2. Run docker
    ```
    docker run -it   --env="DISPLAY"   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"   --device=/dev/dri:/dev/dri    ros_cart
    ```

# Rosbag to rosbag2

To convert `rosbag` bag into `rosbag2`, one can use this command

```
rosbags-convert --src /path/to/dataset.bag --dst /path/to/dataset
```

# Demo

## Official Cartographer example

This section is an adaptation to `ros2` of the [official cartographer guide](https://google-cartographer-ros.readthedocs.io/en/latest/demos.html)


- Download 2D/3D `.bag` sequence. 
Note: unfortunately, oficial links are not available now (31.10.2024). However, one can find some sequences available on [web archive](http://web.archive.org/web/20170110062030/https://google-cartographer-ros.readthedocs.io/en/latest/data.html). In this tutorial, `b3-2016-02-02-13-32-01.bag` will be used.

- Convert it to ROS2 format as shown [above](#rosbag-to-rosbag2)


### Launch demo
```
ros2 launch cartographer_ros demo_backpack_3d.launch.py bag_filename:=/path/to/b3-2016-02-02-13-32-01
```

# Generate `.pbstream` file

## Demo datasets
```
ros2 launch cartographer_ros offline_backpack_3d.launch.py bag_filenames:=/path/to/b3-2016-02-02-13-32-01
```

## Custom datasets

- Download `.bag` file into container. 
- Convert it to ROS2 format as shown [above](#rosbag-to-rosbag2)

In this example, [uzh_tracking_area_run2.bag](https://storage.googleapis.com/hilti_challenge/uzh_tracking_area_run2.bag) will be used.

_Note_: this repo additionally contains config and launch files for [Basement_1.bag](https://storage.googleapis.com/hilti_challenge/Basement_1.bag)

### Manually 2D

- Start cartographer 
    ```
    ros2 launch cartographer_ros uzh_tracking_area_run2_2D.launch.py bag_filename:=/path/to/uzh_tracking_area_run2
    ```
- To save the map, open another terminal inside docker and execute
    ```
    ros2 run nav2_map_server map_saver_cli -f map
    ```

### Manually 3D

TODO: fix the fact that the map has very low quality

- Start cartographer 
    ```
    ros2 launch cartographer_ros uzh_tracking_area_run2_3D.launch.py bag_filename:=/path/to/uzh_tracking_area_run2
    ```

- In another termminal, call `FinishTrajectory` service to save the trajectory
    ```
    ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
    ```

- Wait until the trajectory is finished (check log in the terminal where cartographer is running) and save the trajectory

    ```
    ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/path/to/uzh_tracking_area_run2.pbstream', include_unfinished_submaps: true}"
    ```

### Automatically 3D

For scripts one can use `cartographer_offline_node` to generate `.pbstream` file in one command:

```
ros2 launch cartographer_ros offline_uzh_tracking_area_run2_3D.launch.py bag_filenames:=/path/to/uzh_tracking_area_run2 skip_seconds:=10
```

This will generate `/path/to/uzh_tracking_area_run2.pbstream` file in the same directory.

_Note_: one can specify `skip_seconds` to skip some seconds from the beginning of the dataset.

# `.pbstream` to `.ply`

To generate `.ply` file, use `assets_writer_3d.launch.py` for hilti example
```
ros2 launch cartographer_ros assets_writer_3d.launch.py bag_filenames:=/path/to/uzh_tracking_area_run2 pose_graph_filename:=/path/to/uzh_tracking_area_run2.pbstream
```

or `assets_writer_3d_with_urdf.launch.py` for cartographer example
```
ros2 launch cartographer_ros assets_writer_3d_with_urdf.launch.py bag_filenames:=/workspace/bags/b3-2016-02-02-13-32-01 pose_graph_filename:=/workspace/bags/b3-2016-02-02-13-32-01.pbstream
```

For other bag files one may need to create custom launch file

# Map visualization
- Visualize `.ply` file on host using [meshlab](https://www.meshlab.net/#download) or [point_cloud_viewer](/point_cloud_viewer/)


# Map quality estimation

> **Note**: As mentioned in [Cartographer documentation](https://google-cartographer.readthedocs.io/en/latest/evaluation.html#advantages-limitations), this estimation is valid only for local SLAM estimations with optimizations disabled. 

Generate optimized map as described in [Generate pbstream](#generate-pbstream) section. Further we will assume that the file is named `uzh_tracking_area_run2_map.pbstream`.

Next, one can autogenerate ground truth by running:
```
cartographer_autogenerate_ground_truth \
    -pose_graph_filename /path/to/uzh_tracking_area_run2_map.pbstream \
    -output_filename /path/to/uzh_tracking_area_run2_relations.pbstream \
    -min_covered_distance 20 \
    -outlier_threshold_meters 0.15 \
    -outlier_threshold_radians 0.02
```

Then, turn off optimization by setting `POSE_GRAPH.optimize_every_n_nodes = 0` in `.lua` configuration file and generate relations again as described above, and launch cartographer with this configuration (as described above).

Next, save the file into `.pbstream` format using `cartographer_save_map` tool as described in [Generate pbstream](#generate-pbstream) section but skip optimization step. We will assume it is named `uzh_tracking_area_run2_test.pbstream`.

Finally, you will have 3 files:
- `uzh_tracking_area_run2_map.pbstream` test map
- `uzh_tracking_area_run2_relations.pbstream` relations
- `uzh_tracking_area_run2_test.pbstream` unoptimized map

To estimate local SLAM quality run: 

```
cartographer_compute_relations_metrics \
    -relations_filename /path/to/uzh_tracking_area_run2_relations.pbstream \
    -pose_graph_filename /path/to/uzh_tracking_area_run2_test.pbstream
```

Example output: 
```
Abs translational error 0.26454 +/- 0.13541 m
Sqr translational error 0.08812 +/- 0.08872 m^2
Abs rotational error 1.80877 +/- 1.01483 deg
Sqr rotational error 4.29034 +/- 4.92501 deg^2
```