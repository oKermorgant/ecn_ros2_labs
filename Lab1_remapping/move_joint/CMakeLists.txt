cmake_minimum_required(VERSION 3.5)
project(move_joint)

set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_auto_add_executable(move_joint src/move_joint.cpp)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_auto_package()
