cmake_minimum_required(VERSION 3.5)
# TODO give a name to your project
project(ecn_turtle)

set(CMAKE_CXX_STANDARD 17)

# TODO find necessary libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(turtlesim REQUIRED)
find_package(rclcpp_components REQUIRED)

# node as an executable
add_library(follower SHARED follower.cpp)


# TODO build the node as a library
#add_library(<your_node> SHARED <required_files>)

# TODO include non-ROS2 directories (OpenCV, Yaml-cpp, etc.)
#target_include_directories(<your_node> PRIVATE ${YamlCpp_INCLUDE_DIRS})

# TODO link your node to its ROS 2 dependencies
ament_target_dependencies(follower rclcpp geometry_msgs turtlesim rclcpp_components)
  
# TODO let ament create an executable from your node
rclcpp_components_register_node(follower
  PLUGIN "ecn_turtle::Follower"
  EXECUTABLE follower_node)

# install things
install(TARGETS follower
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
  
# install other directories if needed
#install(DIRECTORY launch urdf maps params
  #DESTINATION share/${PROJECT_NAME})

# register node
ament_export_libraries(follower)
ament_package()
