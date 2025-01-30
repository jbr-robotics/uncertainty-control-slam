# Example cartographer launch

# TODO: this document is under development

## Enviroment preparation
0. Enter [cartographer_example](../cartographer_example/) directory in your trerminal:
    ```
    cd path/to/uncertainty-control-slam/cartographer_example
    ```

1. Build docker:
    ```
    docker build -t cartographer_example .
    ```
2. Do not forget to allow X-forwarding using `xhost`:
    ```
    xhost +local:docker
    ```
3. Create data direcory:
    ```
    mkdir /path/to/data/direcotry
    ```
3. Run docker (do not forget to change `/path/to/data/direcory` to the valid value):
    ```
    docker run -it \
        --env="DISPLAY" \
        --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
        --device=/dev/dri:/dev/dri \
        --volume="/path/to/data/direcotry:/data" \
        --volume="${PWD}/configuration_files/uzh_tracking_area_run2_3D.lua:/opt/ros/humble/share/cartographer_ros/configuration_files/uzh_tracking_area_run2_3D.lua" \
        cartographer_example
    ```

> _Note_: do further steps inside the docker container

## Data preparation

Download the `uzh_tracking_area_run2.bag` file and convert it to rosbag2 format:
```
cd /data \
&& wget https://storage.googleapis.com/hilti_challenge/uzh_tracking_area_run2.bag \
&& rosbags-convert --src ${PWD}/uzh_tracking_area_run2.bag --dst ${PWD}/uzh_tracking_area_run2
```

## Launch cartographer

This section will explain how to luanch cartographer in two modes:

- [Online](#launch-online-cartographer) which gives an opportuninty to visualize the navigation and map building process

- [Offline](#launch-offline-cartographer) which gives less control over ...

TODO: modify offline mode to enable RVIZ and then delete online cartographer configuration from the example

### Launch Online Cartographer

1. Start cartographer:
    ```
    ros2 launch cartographer_ros uzh_tracking_area_run2_3D.launch.py bag_filename:=/data/uzh_tracking_area_run2
    ```

    - It will open rviz, allowing you observe the map building process. 

2. Wait until the message pops up:
    ```
    TODO
    ```

3. Now, you are ready to finish the trajecory before saving it: using **2n termminal**, call `FinishTrajectory` service.
    ```
    ros2 service call /finish_trajectory cartographer_ros_msgs/srv/FinishTrajectory "{trajectory_id: 0}"
    ```
4.  Get back to the **1st terminal** and wait for a message indicating sucessfull
- Wait until the trajectory is finished (check log in the terminal where cartographer is running) and save the trajectory

    ```
    ros2 service call /write_state cartographer_ros_msgs/srv/WriteState "{filename: '/data/uzh_tracking_area_run2.pbstream', include_unfinished_submaps: true}"
    ```

### Launch offline Cartographer

For simplicity, one can automate the process described above and build a map in one command: 
```

```