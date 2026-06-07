cmake_minimum_required(VERSION 3.8)
project(fastbot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/WaypointAction.action"
  DEPENDENCIES geometry_msgs
)

# Action server executable
add_executable(fastbot_action_server src/fastbot_action_server.cpp)
ament_target_dependencies(fastbot_action_server
  rclcpp rclcpp_action geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)
rosidl_target_interfaces(fastbot_action_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS fastbot_action_server
  DESTINATION lib/${PROJECT_NAME})

install(
  PROGRAMS scripts/pose_marker_display.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  # ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_waypoints
    test/test_waypoints.cpp
    test/main.cpp
    SKIP_LINKING_MAIN_LIBRARIES)
  target_link_libraries(test_waypoints gtest)
  ament_target_dependencies(test_waypoints
    rclcpp rclcpp_action geometry_msgs nav_msgs tf2 tf2_ros tf2_geometry_msgs)
  rosidl_target_interfaces(test_waypoints
    ${PROJECT_NAME} "rosidl_typesupport_cpp")
endif()

ament_package()