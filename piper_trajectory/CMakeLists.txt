cmake_minimum_required(VERSION 3.8)
project(piper_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(tictactoe_trajectory src/tictactoe_trajectory.cpp)

ament_target_dependencies(tictactoe_trajectory
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  tf2_geometry_msgs
)

install(TARGETS tictactoe_trajectory
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
