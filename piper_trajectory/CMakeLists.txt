cmake_minimum_required(VERSION 3.8)
project(piper_trajectory)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ExecuteTrajectory.srv"
)

include_directories(
  include
)

add_library(trajectory_executor
  src/trajectory_executor.cpp
)

ament_target_dependencies(trajectory_executor
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  tf2_geometry_msgs
)

rosidl_target_interfaces(trajectory_executor
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

add_executable(trajectory_server
  src/trajectory_server.cpp
)

ament_target_dependencies(trajectory_server
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
  tf2_geometry_msgs
)

target_link_libraries(trajectory_server
  trajectory_executor
)

rosidl_target_interfaces(trajectory_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(
  TARGETS trajectory_executor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
)

install(
  TARGETS trajectory_server
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include/
)

install(
  DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

ament_package()
