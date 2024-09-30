1. Build docker `docker build -t ros .`
2. Run docker `docker run -it ros`
3. Install `.bag`file into container https://storage.googleapis.com/hilti_challenge/uzh_tracking_area_run2.bag
4. Run cartographer: 
```
roslaunch cartographer_ros my_demo_backpack_2d.launch bag_filename:=/path/to/uzh_tracking_area_run2.bag
```
5. Do not close cartographer and save the map using another terminal:

```
docker exec -it <CONTAINER ID> /bin/bash
rosrun map_server map_saver -f my_map
```
