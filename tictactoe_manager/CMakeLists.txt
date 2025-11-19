cmake_minimum_required(VERSION 3.8)
project(tictactoe_manager)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(board_perception REQUIRED)
find_package(piper_trajectory REQUIRED)

add_executable(tictactoe_manager
  src/tictactoe_manager.cpp
)

ament_target_dependencies(tictactoe_manager
  rclcpp
  board_perception
  piper_trajectory
)

install(
  TARGETS tictactoe_manager
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()
