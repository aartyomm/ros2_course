cmake_minimum_required(VERSION 3.8)
project(turtle_controller_pkg)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

set(PYTHON_NODES
  turtle_mover
  pose_listener
)

foreach(NODE ${PYTHON_NODES})
  install(PROGRAMS
    turtle_controller_pkg/${NODE}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${NODE}
  )
endforeach()

ament_package()
