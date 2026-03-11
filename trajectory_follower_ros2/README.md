<!-- Purpose: Project overview, build/run instructions, and design notes. -->
# Trajectory Follower ROS2

This package implements path smoothing, time-parameterized trajectory generation, and a DWA-based trajectory tracking controller for a differential drive robot.

## Features
- Cubic spline path smoothing from waypoints
- Time-parameterized trajectory (trapezoidal velocity profile)
- DWA local planner for obstacle avoidance
- RViz visualization: planned path, executed path, and input waypoints

## Build
```bash
colcon build --packages-select trajectory_follower_ros2
source install/setup.bash
```

## Run (Gazebo + RViz + Planner)
```bash
ros2 launch trajectory_follower_ros2 robot_nav_launch.py
```

## Send Waypoints
A new terminal is launched automatically (or run manually):
```bash
ros2 run trajectory_follower_ros2 trajectory_action_client
```
Example input:
```
1,1 6,7 8,8 -1,5
```

## Topics
- `/smooth_path` – smoothed path (Path)
- `/smooth_path_from_robot` – smoothed path anchored at start pose (Path)
- `/executed_path` – actual robot trail (Path)
- `/input_waypoints` – raw input waypoints (PoseArray)
- `/goal_pose` – final goal for DWA (PoseStamped)
- `/timed_path` – time-parameterized path (Path with stamps)

## Design Choices
- Cubic spline for smooth, continuous paths.
- Trapezoidal velocity profile for time-parameterization.
- DWA for local obstacle avoidance and tracking of the smoothed path.

## Real Robot Extension
- Replace Gazebo odom with a fused state estimate (EKF/SLAM).
- Add safety checks on cmd_vel and watchdog timeouts.
- Use a global planner for obstacle-aware global routes.

## AI Tools Used
- ChatGPT/Codex for code review, refactoring, and documentation.

## Extra Credit: Obstacle Avoidance Extension
- Current DWA is a local planner using laser scan obstacles.
- To extend, add a global planner (A*/Dijkstra) over a costmap.

## Parameters
`trajectory_action_server`:
- `traj_max_vel` (default: 0.4)
- `traj_max_acc` (default: 0.6)
