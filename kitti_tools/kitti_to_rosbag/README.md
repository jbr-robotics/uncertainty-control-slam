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
./kitti_to_rosbag.sh /data/kitti/2011_09_26 /data/kitti/2011_09_26/2011_09_26_drive_0002_sync /data/kitti/2011_09_26_bag
```

# How to download Kitti dataset

Below is presentet an example of how to download Kitti dataset and convert it to rosbag.

Seek sequences at official website: https://www.cvlibs.net/datasets/kitti/raw_data.php

1. Get calibration data. 

    Note: paste the link relevant to your selected sequence
    ```bash
    wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_calib.zip
    unzip 2011_09_26_calib.zip
```

2. Get sequence data (synced + rectified data). 

    Note: paste the link relevant to your selected sequence
    ```bash
    wget https://s3.eu-central-1.amazonaws.com/avg-kitti/raw_data/2011_09_26_drive_0017/2011_09_26_drive_0017_sync.zip
    unzip 2011_09_26_drive_0017_sync.zip
    ```

3. Steps above will produce a directory with calibration data and sequence data.

