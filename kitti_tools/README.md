# Kitti Tools

This subdirectory contains tools for processing KITTI datasets.

> Note:
> It is recommended to use [kitti2bag](./kitti2bag) tool rather than [kitti_to_rosbag](./kitti_to_rosbag) tool.


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

