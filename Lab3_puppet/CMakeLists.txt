cmake_minimum_required(VERSION 3.5)
project(lab3_puppet)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Debug)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(baxter_core_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rclcpp_components REQUIRED)

# source file for lab3: just a C++ class
add_library(puppet_node SHARED puppet_node.cpp ik_client.h)
ament_target_dependencies(puppet_node rclcpp baxter_core_msgs tf2_ros rclcpp_components)

# auto-executable from node
rclcpp_components_register_node(puppet_node
  PLUGIN "lab3_puppet::PuppetNode"
  EXECUTABLE puppet)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(TARGETS puppet_node RUNTIME DESTINATION lib/${PROJECT_NAME})
  
ament_package()
