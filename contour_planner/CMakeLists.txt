cmake_minimum_required(VERSION 3.8)
project(contour_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenCV REQUIRED)
find_package(piper_trajectory REQUIRED)

add_executable(contour_planner src/contour_planner.cpp)
ament_target_dependencies(contour_planner rclcpp OpenCV piper_trajectory)

target_include_directories(contour_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

install(TARGETS
  contour_planner
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY images/
  DESTINATION share/${PROJECT_NAME}/images
)

ament_package()
