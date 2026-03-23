cmake_minimum_required(VERSION 3.8)
project(my_robot_pkg)


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SensorData.msg"
  "srv/ComputeDistance.srv"
  "action/Navigate.action"
)

install(PROGRAMS
  my_robot_pkg/sensor_publisher.py
  RENAME sensor_pub
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  my_robot_pkg/sensor_subscriber.py
  RENAME sensor_sub
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  my_robot_pkg/distance_client.py
  RENAME distance_cli
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  my_robot_pkg/distance_server.py
  RENAME distance_serv
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  my_robot_pkg/navigate_client.py
  RENAME navigate_cli
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  my_robot_pkg/navigate_server.py
  RENAME navigate_serv
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
