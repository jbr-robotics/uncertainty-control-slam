This page is based on the [doc](https://google-cartographer-ros.readthedocs.io/en/latest/assets_writer.html)

## Overview
`.pbstream` is a serialization of the Cartographer internal state: compressed [protobuf](https://en.wikipedia.org/wiki/Protocol_Buffers)

Real-time mapping is very rough as most of the sensor data is thrown away. However, cartographer allows to create a high resolution map a posteriori using `cartographer_assets_writer`

## Input
1. `bag` file with the original sensors data. 
2. A cartographer state: `.pbstream` 
3. TF data from the bag or an URDF description
4. Configuration: `.lua`

## Launch
```
cartographer_node = Node(
        package = 'cartographer_ros',
        executable = 'cartographer_assets_writer',
        parameters = [{'use_sim_time': False}],
        arguments = [
            '-configuration_directory', LaunchConfiguration('configuration_directory'),
            '-configuration_basename', LaunchConfiguration('config_file'),
            '-urdf_filename', LaunchConfiguration('urdf_filename'),
            '-bag_filenames', LaunchConfiguration('bag_filenames'),
            '-pose_graph_filename', LaunchConfiguration('pose_graph_filename')],
        output = 'screen'
        )
```

## Configuration
Simplest configuration that saves map as `.ply` file
```
VOXEL_SIZE = 5e-2

include "transform.lua"

options = {
    tracking_frame = "base_link",
    pipeline = {
        {
        action = "write_ply",
        filename = "trajectory.ply",
        },
    },
}
  
return options
```

## Visualization
Use [point_cloud_viewer](https://github.com/cartographer-project/point_cloud_viewer) or [meshlab](http://www.meshlab.net/) to visualize resulting `.ply` file