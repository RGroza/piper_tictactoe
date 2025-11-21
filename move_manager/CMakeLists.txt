cmake_minimum_required(VERSION 3.8)
project(move_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(board_perception REQUIRED)
find_package(piper_trajectory REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/PlayMove.srv"
)

ament_export_dependencies(
  rosidl_default_runtime
)

add_executable(move_manager_node
  src/move_manager.cpp
)

ament_target_dependencies(move_manager_node
  rclcpp
  board_perception
  piper_trajectory
  rosidl_default_runtime
)

rosidl_target_interfaces(move_manager_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

install(
  TARGETS move_manager_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY srv/
  DESTINATION share/${PROJECT_NAME}/srv
)

ament_package()
