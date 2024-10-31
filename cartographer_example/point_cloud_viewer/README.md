# Point Cloud Viewer

This directory contains Dockerfile to build https://github.com/cartographer-project/point_cloud_viewer and scripts to use it

# Build docker

```
./build_docker.sh
```

# Create octree

```
./build_octree.sh /path/to/input/map.ply /path/to/octree/directory [--resolution <resolution>] [--num-threads <num-threads>]"
```

#  SDL viewer

```
./sdl_viewer.sh /path/to/octree/directory
```

Keys:

| Key                | Action                        |
| ------------------ | ----------------------------- |
| T                  | Toggle the view to CT mode    |
| W                  | Move forward                  |
| A                  | Move left                     |
| S                  | Move backwards                |
| D                  | Move right                    |
| Q                  | Move up                       |
| Z                  | Move down                     |
| Up                 | Turn up                       |
| Left               | Turn left                     |
| Down               | Turn down                     |
| Right              | Move right                    |
| 0                  | Increase points size          |
| 9                  | Decrease points size          |
| 8                  | Brighten scene                |
| 7                  | Darken scene                  |
| O                  | Show octree nodes             |
| Shift + Ctrl + 0-9 | Save current camera position. |
| Ctrl + 0-9         | Load saved camera position.   |
