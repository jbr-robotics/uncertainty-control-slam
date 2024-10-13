11. Build docker `docker build -t ros_cart .`
2. Run docker
    ```
    docker run -it   --env="DISPLAY"   --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw"   --device=/dev/dri:/dev/dri    ros_cart
    ```
3. Install `.bag`file into container https://storage.googleapis.com/hilti_challenge/uzh_tracking_area_run2.bag
4. Covert it to ROS2 format:
    ```
    rosbags-convert --src /path/to/uzh_tracking_area_run2.bag --dst /path/to/uzh_tracking_area_run2
    ```
4. Run cartographer: 
    ```
    ros2 launch cartographer_ros my_2d.py bag_filename:=/path/to/uzh_tracking_area_run2.bag
    ```
5. Do not close cartographer and save the map using another terminal:
    ```
    docker exec -it <CONTAINER ID> /bin/bash
    rosrun map_server map_saver -f my_map
    ```
