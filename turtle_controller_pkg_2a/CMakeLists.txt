cmake_minimum_required(VERSION 3.8)
project(turtle_controller_pkg)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TurtleState.msg"
  "srv/SpawnTurtle.srv"
  "srv/ChangeBackground.srv"
  "srv/RemoveTurtle.srv"
)

set(PYTHON_NODES
  turtle_mover
  pose_listener
  spawn_server
  spawn_client
  background_client
  remove_client
)

foreach(NODE ${PYTHON_NODES})
  install(PROGRAMS
    turtle_controller_pkg/${NODE}.py
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${NODE}
  )
endforeach()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
