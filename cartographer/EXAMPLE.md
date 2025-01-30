# Example of Cartographer Usage

This document provides instructions for using Cartographer with the `uzh_tracking_area_run2` sequence, also known as the "RPG Drone Testing Arena," from the [Hilti dataset](https://www.hilti-challenge.com/dataset-2021.html).

## Data Preparation

Download the `uzh_tracking_area_run2.bag` file and convert it to the ROS 2 bag format:

```sh
cd /data \
&& wget https://storage.googleapis.com/hilti_challenge/uzh_tracking_area_run2.bag \
&& rosbags-convert --src ${PWD}/uzh_tracking_area_run2.bag --dst ${PWD}/uzh_tracking_area_run2
```

## Online Cartographer

> **Note:** If working with pre-recorded datasets, it is recommended to skip this section and proceed to [Offline Cartographer](#offline-cartographer).

1. Start Cartographer:
    ```sh
    ros2 launch cartographer_ros uzh_tracking_area_run2_3D.launch.py bag_filename:=/data/uzh_tracking_area_run2
    ```
    This will open RViz, allowing you to observe the map-building process.

2. Wait for the ROS bag to finish processing. The following log message will indicate completion:
    ```sh
    [INFO] [playbag-4]: process has finished cleanly [pid 63]
    ```
    Additionally, you will notice that the image in RViz freezes, signaling that playback has ended.

3. Before saving the trajectory, you must finish it. In a **second terminal**, call the `FinishTrajectory` service:
    ```sh
    ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
    ```

4. Return to the **first terminal** and wait for a confirmation message indicating a successful trajectory finish.

5. Once optimization is complete, save the trajectory using the **second terminal**:
    ```sh
    ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/data/uzh_tracking_area_run2.pbstream', include_unfinished_submaps: true}"
    ```

6. After saving the map, terminate the online Cartographer process by pressing `Ctrl + C`.

_Optional:_ You can also save a 2D map using the following command:
```sh
ros2 run nav2_map_server map_saver_cli -f map
```

## Offline Cartographer

> **Note:** In this mode, the bag's topics will not be published, which limits debugging capabilities. 

To simplify the process, you can automate map building with a single command:

```sh
ros2 launch cartographer_ros offline_uzh_tracking_area_run2_3D.launch.py bag_filenames:=/data/uzh_tracking_area_run2 skip_seconds:=0 no_rviz:=false
```

This will generate the `/data/uzh_tracking_area_run2.pbstream` file.

_Optional:_ You can use the `skip_seconds` parameter to exclude the initial seconds of the dataset from processing.

## Exporting the Map to `.ply`

To generate a `.ply` file, use the `assets_writer_3d.launch.py` script:

```sh
ros2 launch cartographer_ros assets_writer_3d.launch.py bag_filenames:=/data/uzh_tracking_area_run2 pose_graph_filename:=/data/uzh_tracking_area_run2.pbstream
```

# Other Datasets

This section contains notes on using different datasets with Cartographer.

## Official Cartographer Samples

For official Cartographer datasets, use a different asset writer:

```sh
ros2 launch cartographer_ros assets_writer_3d_with_urdf.launch.py bag_filenames:=/workspace/bags/b3-2016-02-02-13-32-01 pose_graph_filename:=/workspace/bags/b3-2016-02-02-13-32-01.pbstream
```

## Hilti Dataset

The example provided in this document is based on the Hilti dataset. Minimal modifications are required to use other Hilti sequences. Configuration files for [Basement_1.bag](https://storage.googleapis.com/hilti_challenge/Basement_1.bag) can be found in this repository.

## KITTI Dataset

This repository also includes configuration files for the KITTI dataset.

To use KITTI data, convert it into a `.bag` file using [kitti2bag](../kitti_tools/kitti2bag/) and then convert it to the ROS 2 bag format.

