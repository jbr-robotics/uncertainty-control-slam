## Preparation

1. Build docker `docker build -t ros_cart .`
2. Run docker
    ```
    docker run -it   --env="DISPLAY"   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"   --device=/dev/dri:/dev/dri    ros_cart
    ```
3. Install `.bag` file into container https://storage.googleapis.com/hilti_challenge/uzh_tracking_area_run2.bag
4. Covert it to ROS2 format:
    ```
    rosbags-convert --src /path/to/uzh_tracking_area_run2.bag --dst /path/to/uzh_tracking_area_run2
    ```

## 2D

1. Start cartographer 
    ```
    ros2 launch cartographer_ros my_2d.py bag_filename:=/path/to/uzh_tracking_area_run2
    ```
2. To save the map, open another terminal inside docker and execute
    ```
    ros2 run nav2_map_server map_saver_cli -f map
    ```

## 3D 

1. Start cartographer 
    ```
    ros2 launch cartographer_ros my_3d.py bag_filename:=/path/to/uzh_tracking_area_run2
    ```

3. In another termminal save the trajectory
    ```
    ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"

    ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/path/to/uzh_tracking_area_run2.pbstream', include_unfinished_submaps: true}"

    ```

4. Save `uzh_tracking_area_run2_trajectory.ply` file:
    ```
    ros2 launch cartographer_ros assets_writer_3d.py \
        bag_filenames:=/path/to/uzh_tracking_area_run2 \
        pose_graph_filename:=/path/to/uzh_tracking_area_run2.pbstream
    ```

5. Visualize it on host using [meshlab](https://www.meshlab.net/#download)


