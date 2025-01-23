# Kitti_to_rosbag

The subdirectory contains simple docker wrapper for https://github.com/ethz-asl/kitti_to_rosbag

# Installation 

```bash
./build_docker.sh
```

# Usage

```bash
./kitti_to_rosbag.sh <calibration_path> <dataset_path> <output_path>
```

Example usage:

```bash
./kitti_to_rosbag.sh /data/kitty/2011_09_26 /data/kitty/2011_09_26/2011_09_26_drive_0002_sync /data/kitty/2011_09_26_bag
```
