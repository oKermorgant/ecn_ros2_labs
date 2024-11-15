cmake_minimum_required(VERSION 3.5)
project(lab5_gazebo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf launch config DESTINATION share/${PROJECT_NAME})
  
ament_package()
