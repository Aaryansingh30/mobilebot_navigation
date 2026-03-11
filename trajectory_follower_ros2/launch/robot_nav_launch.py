# Purpose: Launch Gazebo, RViz, action server, DWA planner, and optional client.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Base launch wiring: Gazebo + robot + planner + visualization + client.
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_desc_pkg = get_package_share_directory("robot_description")

    model_arg = DeclareLaunchArgument(
        name="model",
        default_value=os.path.join(
            robot_desc_pkg, "urdf", "mobilerobot.urdf.xacro"
        ),
        description="Absolute path to robot URDF file",
    )

    world_arg = DeclareLaunchArgument(
        name="world",
        default_value=os.path.join(
            robot_desc_pkg, "worlds", "mobilerobot_world.world"
        ),
        description="Gazebo world file",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="true",
        description="Use simulation time",
    )
    start_client_arg = DeclareLaunchArgument(
        name="start_client",
        default_value="true",
        description="Start action client in a new terminal",
    )

    robot_description = ParameterValue(
        Command(["xacro ", LaunchConfiguration("model")]),
        value_type=str,
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("gazebo_ros"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={"world": LaunchConfiguration("world")}.items(),
    )

    # Publish robot_description for Gazebo + RViz.
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim_time}],
        output="screen",
    )

    # Spawn robot model into Gazebo.
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=["-entity", "MobileBot", "-topic", "/robot_description"],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("trajectory_follower_ros2"), "rviz", "trajectory_nav.rviz"]
    )
    # RViz with preconfigured displays.
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_config],
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Action server for smoothing and time-parameterization.
    trajectory_server = Node(
        package="trajectory_follower_ros2",
        executable="trajectory_action_server",
        name="trajectory_action_server",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # DWA planner node for obstacle avoidance.
    dwa_node = Node(
        package="trajectory_follower_ros2",
        executable="obs_avoidance_dwa",
        name="obs_avoidance_dwa",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    # Optional interactive client in a new terminal.
    client_terminal = ExecuteProcess(
        cmd=[
            "gnome-terminal", "--", "ros2", "run",
            "trajectory_follower_ros2", "trajectory_action_client"
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("start_client")),
    )

    # Logs tracking error vs time-parameterized trajectory.
    tracking_monitor = Node(
        package="trajectory_follower_ros2",
        executable="tracking_error_monitor",
        name="tracking_error_monitor",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
    )

    return LaunchDescription(
        [
            model_arg,
            world_arg,
            use_sim_time_arg,
            start_client_arg,
            gazebo_launch,
            robot_state_publisher_node,
            spawn_entity,
            rviz_node,
            trajectory_server,
            dwa_node,
            tracking_monitor,
            client_terminal,
        ]
    )
