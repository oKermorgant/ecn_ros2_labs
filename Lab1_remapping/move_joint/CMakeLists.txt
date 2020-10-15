cmake_minimum_required(VERSION 3.5)
project(move_joint)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(baxter_core_msgs REQUIRED)

add_executable(move_joint src/move_joint.cpp)
ament_target_dependencies(
  move_joint rclcpp std_msgs baxter_core_msgs)

install(TARGETS move_joint DESTINATION lib/${PROJECT_NAME})

ament_package()

