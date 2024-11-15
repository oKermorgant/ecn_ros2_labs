cmake_minimum_required(VERSION 3.5)
project(lab4_navigation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE Release)

find_package(ament_cmake REQUIRED)

# nothing to actually compile

install(PROGRAMS src/vel2joints.py src/follow_path.py
  DESTINATION lib/${PROJECT_NAME})
  
install(DIRECTORY launch urdf maps params DESTINATION share/${PROJECT_NAME})
  
ament_package()
