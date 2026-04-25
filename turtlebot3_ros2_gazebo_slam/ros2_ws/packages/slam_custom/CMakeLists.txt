cmake_minimum_required(VERSION 3.8)
project(slam_custom)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  slam_custom/slam_custom.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
