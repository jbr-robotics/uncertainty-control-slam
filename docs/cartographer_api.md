This page is based on the [doc](https://google-cartographer-ros.readthedocs.io/en/latest/ros_api.html)

# Input 

## Range Data
- `points2`/ [PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html) 
- `echos` / [MultiEchoLaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/MultiEchoLaserScan.html)
- `scan` / [LaserScan](http://docs.ros.org/api/sensor_msgs/html/msg/LaserScan.html)

## Additional data
- `imu` / [Imu](http://docs.ros.org/api/sensor_msgs/html/msg/Imu.html)
  Mandatory in `3D`
- `odom` / [Odometry](http://docs.ros.org/api/nav_msgs/html/msg/Odometry.html)
  Optional
  `use_odometry = false` to use the topic
  
# Output 

## Published topics

- `scan_matched_points2` / [PointCloud2](http://docs.ros.org/api/sensor_msgs/html/msg/PointCloud2.html)
	
    Processed LiDAR data from `/points2`; the points are filtered and aligned with the map.
- `submap_list` / [SubmapList](https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/msg/SubmapList.msg)
  
  The list of available sub-maps: index, position, is_frozen. A specific map can be fetched via `/submap_quiry` service
- `tracked_pose` / [PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html)
  
  Optional, `publish_tracked_pose = true` must be set to use. The pose of a tracking frame 


# Services

1. `submap_query` / [SubmapQuery](https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/srv/SubmapQuery.srv)
   
   Returns a sub-map as a matrix with densities. 
	```
   ros2 service call /submap_query cartographer_ros_msgs/srv/SubmapQuery "{trajectory_id: 0, submap_index: 0}"
    ```
2. `start_trajectory` / [StartTrajectory](https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/srv/StartTrajectory.srv)
3. `trajectory_query` / [TrajectoryQuery](https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/srv/TrajectoryQuery.srv)
   
   Returns a list of positions: `position = (x, y, z), orientation = (x, y, z, w)`
	```
	ros2 service call /trajectory_query cartographer_ros_msgs/srv/TrajectoryQuery "{trajectory_id: 0}"
   ```
4. `finish_trajectory` / [FinishTrajectory](https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/srv/FinishTrajectory.srv)
   
   Finishes an ongoing trajectory, which triggers a final optimization pass. 
   ```
    ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
	```
5. `write_state` / [WriteState](https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/srv/WriteState.srv)
   
   Writes the current internal state to disk into filename.
   ```
   `ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/home/user/cartographer_state.pbstream', include_unfinished_submaps: true}"
   ```
6. `get_trajectory_states` / [GetTrajectoryStates](https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/srv/GetTrajectoryStates.srv)
7. `read_metrics` / [cartographer_ros_msgs/ReadMetrics](https://github.com/cartographer-project/cartographer_ros/blob/master/cartographer_ros_msgs/srv/ReadMetrics.srv)
   
   Returns metrics some internal Cartographer metrics. Use `--collect_metrics` when starting node to make the service available.

# Required TF transforms
![](https://google-cartographer-ros.readthedocs.io/en/latest/_images/frames_demo_2d.jpg)

- The robot must provide a `base_link` and its transformation to `imu_link`.
- `map_frame = "map"` and `odom_frame = "odom"` specify the names of the map and odom frames.
- `odom` must be provided by the robot. If not, set `provide_odom_frame = true` to force Cartographer to create the `odom` frame.
