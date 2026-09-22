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

install(FILES scripts/yellow_ellipse.py scripts/hand_stick.py DESTINATION lib/${PROJECT_NAME})

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

# Keep the conventional ROS 2 executable spelling as well as the source-file
# spelling above.  This makes `ros2 run ... check_yolo_env` work directly.
install(PROGRAMS
  scripts/check_yolo_env.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME check_yolo_env
)

add_executable(collection_fsm_node src/collection_fsm_node.cpp)
target_include_directories(collection_fsm_node PRIVATE include)
target_compile_features(collection_fsm_node PUBLIC cxx_std_17)
ament_target_dependencies(collection_fsm_node geometry_msgs rclcpp std_msgs mavros_msgs)

install(TARGETS
  collection_fsm_node
  bbox_controller_node
  mission_state_machine_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_executable(test_collection_servo test/test_collection_servo.cpp)
  target_include_directories(test_collection_servo PRIVATE include)
  target_compile_features(test_collection_servo PUBLIC cxx_std_17)
  add_test(NAME collection_servo COMMAND test_collection_servo)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME vision_interface_defaults
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_interface_defaults.py)
endif()

ament_package()
