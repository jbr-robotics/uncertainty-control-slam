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

# Demos and pbstream generation

## Official Cartographer example

This section is an adaptation to `ros2` of the [official cartographer guide](https://google-cartographer-ros.readthedocs.io/en/latest/demos.html)


- Download 2D/3D `.bag` sequence. 
Note: unfortunately, oficial links are not available now (31.10.2024). However, one can find some sequences available on [web archive](http://web.archive.org/web/20170110062030/https://google-cartographer-ros.readthedocs.io/en/latest/data.html). In this tutorial, `b3-2016-02-02-13-32-01.bag` will be used.

- Covert it to ROS2 format as shown [above](#rosbag-to-rosbag2)


### Launch demo
```
ros2 launch cartographer_ros demo_backpack_3d.launch.py bag_filename:=/path/to/b3-2016-02-02-13-32-01
```

### Generate pbstream
```
ros2 launch cartographer_ros offline_backpack_3d.launch.py bag_filenames:=/path/to/b3-2016-02-02-13-32-01
```

## Custom `.bag` file
- Download `.bag` file into container. For example: https://storage.googleapis.com/hilti_challenge/uzh_tracking_area_run2.bag
- Covert it to ROS2 format as shown [above](#rosbag-to-rosbag2)


### 2D

- Start cartographer 
    ```
    ros2 launch cartographer_ros uzh_tracking_area_run2_2D.py bag_filename:=/path/to/uzh_tracking_area_run2
    ```
- To save the map, open another terminal inside docker and execute
    ```
    ros2 run nav2_map_server map_saver_cli -f map
    ```

### 3D 

TODO: fix the fact that the map has very low quality

- Start cartographer 
    ```
    ros2 launch cartographer_ros uzh_tracking_area_run2_3D.py bag_filename:=/path/to/uzh_tracking_area_run2
    ```

- In another termminal save the trajectory
    ```
    ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

    ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/path/to/uzh_tracking_area_run2.pbstream', include_unfinished_submaps: true}"
    ```

# Pbstream to ply

To generate `.ply` file, use `assets_writer_3d.py` for hilti example
```
ros2 launch cartographer_ros assets_writer_3d.py bag_filenames:=/path/to/uzh_tracking_area_run2 pose_graph_filename:=/path/to/uzh_tracking_area_run2.pbstream
```

or `assets_writer_3d_with_urdf.py` for cartographer example
```
ros2 launch cartographer_ros assets_writer_3d.py bag_filenames:=/workspace/bags/b3-2016-02-02-13-32-01 pose_graph_filename:=/workspace/bags/b3-2016-02-02-13-32-01.pbstream
```

For other bag files one may need to create custom launch file

# Map visualization
- Visualize `.ply` file on host using [meshlab](https://www.meshlab.net/#download) or [point_cloud_viewer](/point_cloud_viewer/)


