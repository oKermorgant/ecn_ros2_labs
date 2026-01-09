cmake_minimum_required(VERSION 3.5)
project(lab3_puppet)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(baxter_core_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# source file for lab3
add_executable(puppet puppet_node.cpp)
ament_target_dependencies(puppet rclcpp baxter_core_msgs tf2_ros rclcpp_components)


install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS puppet DESTINATION lib/${PROJECT_NAME})
  
ament_package()
