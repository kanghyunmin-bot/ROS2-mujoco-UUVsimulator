cmake_minimum_required(VERSION 3.8)
project(auv_buoy_vision_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

add_executable(bbox_controller_node src/bbox_controller_node.cpp)
target_compile_features(bbox_controller_node PUBLIC cxx_std_17)
ament_target_dependencies(bbox_controller_node rclcpp std_msgs mavros_msgs)

add_executable(mission_state_machine_node src/mission_state_machine_node.cpp)
target_compile_features(mission_state_machine_node PUBLIC cxx_std_17)
ament_target_dependencies(mission_state_machine_node geometry_msgs rclcpp std_msgs mavros_msgs)

install(PROGRAMS
  scripts/yolo_buoy_detector.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_buoy_detector
)

install(PROGRAMS
  scripts/check_yolo_env.py
  scripts/setup_laptop_yolo_env.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  bbox_controller_node
  mission_state_machine_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
