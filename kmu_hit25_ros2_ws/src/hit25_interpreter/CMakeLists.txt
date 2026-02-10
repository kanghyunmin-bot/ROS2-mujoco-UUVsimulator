cmake_minimum_required(VERSION 3.8)
project(hit25_interpreter)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedObject.msg"
  "msg/VisionData.msg"
  "msg/HydrophoneData.msg"
  DEPENDENCIES std_msgs sensor_msgs
)

ament_export_dependencies(rosidl_default_runtime)

install(DIRECTORY launch models
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/vision_interpreter_node.py
  scripts/vision_interpreter_node_right_start.py
  scripts/hydrophone_analyzer_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS start_interpreter.sh
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
