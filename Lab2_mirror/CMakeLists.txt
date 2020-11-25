cmake_minimum_required(VERSION 3.5)
project(lab2_mirror)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(baxter_core_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# source file for lab2: an actual executable
add_executable(mirror_node mirror_node.cpp)
ament_target_dependencies(
  mirror_node rclcpp baxter_core_msgs sensor_msgs)

install(TARGETS mirror_node
  DESTINATION lib/${PROJECT_NAME})
  
ament_package()
