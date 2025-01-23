# Kitti2bag

The subdirectory contains simple docker wrapper for https://github.com/tomas789/kitti2bag

# Installation 

```bash
./build_docker.sh
```

# Usage

```bash
./kitti2bag.sh -t <DATE> -r <RUN> -p <DATA_PATH> <DATA_TYPE>
```

Example usage:

```bash
./kitti2bag.sh -t 2011_09_26 -r 0002 -p /path/to/data raw_synced
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

4. Convert to rosbag
    ```bash
    ./kitti_to_rosbag.sh /data/kitti/2011_09_26 /data/kitti/2011_09_26/2011_09_26_drive_0017_sync /data/kitti/2011_09_26_bag
    ```
