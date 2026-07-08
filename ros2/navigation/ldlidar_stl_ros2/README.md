# LDROBOT LD06 ROS2 Node

This package is trimmed for the LDROBOT LD06 LiDAR used by this project.

## Build

```bash
cd /home/bianbu/k1muse_slam_ros
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

## Run

Start the LD06 node:

```bash
ros2 launch ldlidar_stl_ros2 ld06.launch.py
```

Start the LD06 node with RViz2:

```bash
ros2 launch ldlidar_stl_ros2 viewer_ld06.launch.py
```

## Serial Port

The project udev rule maps the LD06 serial adapter to `/dev/mylidar`.
The standalone launch file publishes `/scan` and uses this path:

```python
{'port_name': '/dev/mylidar'}
```

Install the project rule from
`/home/bianbu/k1muse_slam_ros/src/udev/99-ros2-slam.rules` on the board if
`/dev/mylidar` is missing.

## Integrated K1 Bringup

The main `k1muse_slam_nav _sensors.launch.py` does not expose raw LD06 data
directly as `/scan`. It starts this node on `/scan_raw`, then runs
`k1_scan_self_filter.py` to remove points inside the robot footprint and publish
the filtered scan as `/scan` for Cartographer and Nav2.
