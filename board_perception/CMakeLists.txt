cmake_minimum_required(VERSION 3.8)
project(board_perception)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_index_cpp REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/ProcessBoard.srv"
  "msg/BoardState.msg"
)

include_directories(
  include
)

add_executable(board_processor_server
  src/board_processor_server.cpp
  src/image_processor.cpp
)

add_executable(board_processor_node
  src/board_processor_node.cpp
  src/image_processor.cpp
)

target_include_directories(board_processor_server PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_include_directories(board_processor_node PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

ament_target_dependencies(board_processor_server
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  ament_index_cpp
)

ament_target_dependencies(board_processor_node
  rclcpp
  sensor_msgs
  cv_bridge
  OpenCV
  ament_index_cpp
)

rosidl_target_interfaces(board_processor_server
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

rosidl_target_interfaces(board_processor_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

install(TARGETS board_processor_server board_processor_node
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY images/
  DESTINATION share/${PROJECT_NAME}/images
)

ament_package()
