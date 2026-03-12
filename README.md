# Mobilebot Navigation Workspace

This repository contains two ROS 2 packages:

- `robot_description`: URDF/Xacro, Gazebo worlds, and RViz configs for MobileBot.
- `trajectory_follower_ros2`: Path smoothing, time-parameterized trajectory generation, DWA controller, and visualization.

## Build
```bash
cd /home/aaryan/trajectoryCont_ws
colcon build --packages-select robot_description trajectory_follower_ros2
source install/setup.bash
```

## Run
```bash
ros2 launch trajectory_follower_ros2 robot_nav_launch.py
```

## Send Waypoints
```bash
ros2 run trajectory_follower_ros2 trajectory_action_client
```
Example input:
```
1,1 6,7 8,8 -1,5
```

## Package Docs
- `trajectory_follower_ros2/README.md` contains detailed design and usage notes.
