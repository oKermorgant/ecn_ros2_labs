cmake_minimum_required(VERSION 3.5)
project(lab2_mirror)

set(CMAKE_CXX_STANDARD 17)

# we use ament_auto for this simple package
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# source file for lab2: an actual executable
ament_auto_add_executable(mirror_node mirror_node.cpp)
  
ament_auto_package()
