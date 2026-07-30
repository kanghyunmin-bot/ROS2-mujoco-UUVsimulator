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
  scripts/pinger_detach_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME pinger_detach_monitor
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

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME yolo_pinger_marker_fast_path
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_yolo_pinger_marker_fast_path.py
  )
  add_test(
    NAME yolo_course_buoy_filter
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_yolo_course_buoy_filter.py
  )
  set_tests_properties(yolo_pinger_marker_fast_path PROPERTIES TIMEOUT 20)
  set_tests_properties(yolo_course_buoy_filter PROPERTIES TIMEOUT 20)
endif()

ament_package()
