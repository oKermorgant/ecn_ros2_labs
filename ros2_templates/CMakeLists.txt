cmake_minimum_required(VERSION 3.5)
# TODO give a name to your project
project(<package name>)

set(CMAKE_CXX_STANDARD 14)

# TODO find necessary libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

# TODO build the node as a library
add_library(<your_node> SHARED <required_files>)

# TODO include non-ROS2 directories (OpenCV, Yaml-cpp, etc.)
#target_include_directories(<your_node> PRIVATE ${YamlCpp_INCLUDE_DIRS})

# TODO link your node to its ROS 2 dependencies
ament_target_dependencies(<your_node>
  rclcpp
  std_msgs
  
# TODO let ament create an executable from your node
rclcpp_components_register_node(<your_node>
  PLUGIN "<package name>::<node class name>"
  EXECUTABLE <executable_name>)

# install things
install(TARGETS <your_node>
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
  
# install other directories if needed
install(DIRECTORY launch urdf maps params
  DESTINATION share/${PROJECT_NAME})

# register node
ament_export_libraries(<your_node>)
ament_package()
