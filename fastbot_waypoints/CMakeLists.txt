cmake_minimum_required(VERSION 3.8)
project(fastbot_waypoints)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(fastbot_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
add_executable(fastbot_action_server src/fastbot_action_server.cpp)
ament_target_dependencies(fastbot_action_server rclcpp fastbot_interfaces geometry_msgs nav_msgs tf2 rclcpp_action rclcpp tf2_geometry_msgs)
install(TARGETS fastbot_action_server DESTINATION lib/${PROJECT_NAME})
if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_waypoint_node test/test_waypoint_node.cpp)
  ament_target_dependencies(test_waypoint_node
    rclcpp
    rclcpp_action
    geometry_msgs
    nav_msgs
    fastbot_interfaces
  )
endif()

ament_package()
