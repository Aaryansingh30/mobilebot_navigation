# Purpose: Build rules for trajectory_follower_ros2 package.
cmake_minimum_required(VERSION 3.8)
project(trajectory_follower_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies for ROS2 nodes, messages, and interfaces.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # Unit test for time-parameterization utilities.
  ament_add_gtest(test_time_param test/test_time_param.cpp)
  target_include_directories(test_time_param PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

# Action interface code generation.
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowTrajectory.action"
  DEPENDENCIES geometry_msgs
)
rosidl_get_typesupport_target(rosidl_typesupport_cpp_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

# Action server executable: smoothing + time-parameterization.
add_executable(trajectory_action_server
  src/trajectory_action_server.cpp
  src/cubic_spline.cpp
)
ament_target_dependencies(trajectory_action_server rclcpp rclcpp_action geometry_msgs nav_msgs)
target_include_directories(trajectory_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(trajectory_action_server ${rosidl_typesupport_cpp_target})


# Obstacle avoidance and tracking (DWA).
add_executable(obs_avoidance_dwa src/obs_avoidance_dwa.cpp)
ament_target_dependencies(obs_avoidance_dwa rclcpp geometry_msgs nav_msgs sensor_msgs tf2 visualization_msgs)
target_include_directories(obs_avoidance_dwa PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Interactive client for sending waypoints.
add_executable(trajectory_action_client src/trajectory_action_client.cpp)
ament_target_dependencies(trajectory_action_client rclcpp rclcpp_action geometry_msgs)
target_include_directories(trajectory_action_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(trajectory_action_client ${rosidl_typesupport_cpp_target})

# Tracking error monitor node.
add_executable(tracking_error_monitor src/tracking_error_monitor.cpp)
ament_target_dependencies(tracking_error_monitor rclcpp nav_msgs std_msgs)

# Install runtime targets and share assets.
install(TARGETS
  trajectory_action_server
  obs_avoidance_dwa
  trajectory_action_client
  tracking_error_monitor
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  rviz
  DESTINATION share/${PROJECT_NAME}
)

# Package README for distribution.
install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
