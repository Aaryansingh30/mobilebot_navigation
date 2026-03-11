# Purpose: Build rules for trajectory_follower_ros2 package.
cmake_minimum_required(VERSION 3.8)
project(trajectory_follower_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
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

  ament_add_gtest(test_time_param test/test_time_param.cpp)
  target_include_directories(test_time_param PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  )
endif()

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/FollowTrajectory.action"
  DEPENDENCIES geometry_msgs
)

add_executable(trajectory_action_server
  src/trajectory_action_server.cpp
  src/cubic_spline.cpp
)
ament_target_dependencies(trajectory_action_server rclcpp rclcpp_action geometry_msgs nav_msgs)
target_include_directories(trajectory_action_server PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(trajectory_action_server ${cpp_typesupport_target})

add_executable(pure_pursuit src/pure_pursuit.cpp)
ament_target_dependencies(pure_pursuit rclcpp geometry_msgs nav_msgs)
target_include_directories(pure_pursuit PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(pure_pursuit ${cpp_typesupport_target})

add_executable(obs_avoidance_dwa src/obs_avoidance_dwa.cpp)
ament_target_dependencies(obs_avoidance_dwa rclcpp geometry_msgs nav_msgs sensor_msgs tf2 visualization_msgs)
target_include_directories(obs_avoidance_dwa PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(trajectory_action_client src/trajectory_action_client.cpp)
ament_target_dependencies(trajectory_action_client rclcpp rclcpp_action geometry_msgs)
target_include_directories(trajectory_action_client PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(trajectory_action_client ${cpp_typesupport_target})

add_executable(tracking_error_monitor src/tracking_error_monitor.cpp)
ament_target_dependencies(tracking_error_monitor rclcpp nav_msgs std_msgs)

install(TARGETS
  trajectory_action_server
  pure_pursuit
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

install(FILES
  README.md
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
