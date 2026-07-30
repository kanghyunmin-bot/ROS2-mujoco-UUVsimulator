cmake_minimum_required(VERSION 3.10)
project(hydrophone_ctrl)

find_package(ament_cmake REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/near_zone_line_search_controller.cpp
  src/region_local_gradient_estimator.cpp
  src/region_local_gradient_rviz_visualizer.cpp
  src/start_frame_publisher.cpp
  src/waypoint_homing_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${EIGEN3_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  audio_common_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  rclcpp
  rclcpp_components
  std_msgs
  tf2_ros
  visualization_msgs
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "audio_capture::RegionLocalGradientEstimatorNode"
  EXECUTABLE region_local_gradient_estimator
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "audio_capture::WaypointHomingControllerNode"
  EXECUTABLE waypoint_homing_controller
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "audio_capture::NearZoneLineSearchControllerNode"
  EXECUTABLE near_zone_line_search_controller
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "audio_capture::RegionLocalGradientRvizVisualizerNode"
  EXECUTABLE region_local_gradient_rviz_visualizer
)
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "hydrophone_ctrl::StartFramePublisherNode"
  EXECUTABLE start_frame_publisher
)

install(
  TARGETS
    ${PROJECT_NAME}
    region_local_gradient_estimator
    waypoint_homing_controller
    near_zone_line_search_controller
    region_local_gradient_rviz_visualizer
    start_frame_publisher
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)

ament_package()
