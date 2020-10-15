cmake_minimum_required(VERSION 3.5)
project(capture_key)

set(CMAKE_CXX_STANDARD 14)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


add_executable(capture_key src/capture_key.cpp)
ament_target_dependencies(
  capture_key rclcpp std_msgs)

install(TARGETS capture_key DESTINATION lib/${PROJECT_NAME})

ament_package()

