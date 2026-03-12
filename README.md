# Autonomous Navigation of a MobileBot

Purpose
- Smooth waypoint paths with cubic splines
- Generate a time-parameterized trajectory (trapezoidal profile)
- Track the trajectory with a DWA-based local planner
- Visualize planned path, executed path, and DWA rollouts in RViz

Package Nodes
- `trajectory_action_server`: action server that smooths waypoints, publishes `/smooth_path`, `/smooth_path_from_robot`, `/timed_path`, and `/goal_pose`
- `trajectory_action_client`: interactive client that sends waypoints from stdin
- `obs_avoidance_dwa`: DWA local planner that tracks the path and avoids obstacles
- `tracking_error_monitor`: logs error between `/timed_path` and `/odom`

Build
```bash
mkdir -p robotics_navigation/src
cd /home/aaryan/robotics_navigation_ws
git clone https://github.com/Aaryansingh30/mobilebot_navigation.git
colcon build
source install/setup.bash
```

Run (Gazebo + RViz + planners)
```bash
ros2 launch trajectory_follower_ros2 robot_nav_launch.py
```

Waypoint Input
In the client terminal, enter waypoints as:
```
1,1 6,7 8,8 -1,5
```
Type `quit` to exit the client.

Key Topics
- `/input_waypoints` (geometry_msgs/PoseArray): raw waypoints
- `/smooth_path` (nav_msgs/Path): smoothed path
- `/smooth_path_from_robot` (nav_msgs/Path): smoothed path anchored at start pose
- `/timed_path` (nav_msgs/Path): time-parameterized path
- `/goal_pose` (geometry_msgs/PoseStamped): final goal for DWA
- `/executed_path` (nav_msgs/Path): actual path driven by the robot
- `/dwa_markers` (visualization_msgs/MarkerArray): DWA rollouts and goal marker

Key Parameters
- `traj_max_vel` (trajectory_action_server): max speed for time-parameterization
- `traj_max_acc` (trajectory_action_server): max accel for time-parameterization
- `use_sim_time` (all nodes): simulation time

RViz
- Configuration file: `trajectory_follower_ros2/rviz/trajectory_nav.rviz`
- Fixed frame: `odom`

Design Notes
- Cubic splines provide a smooth, continuous path between waypoints.
- A trapezoidal velocity profile yields time stamps along the path.
- DWA provides local obstacle avoidance while following the smoothed path.

