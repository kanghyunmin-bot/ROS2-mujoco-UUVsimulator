cmake_minimum_required(VERSION 3.8)
project(kmu26_auv_surface_buoy_mission)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(auv_msg REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(surface_lane_planner src/lane_planner.cpp)
target_compile_features(surface_lane_planner PUBLIC cxx_std_17)
target_include_directories(
  surface_lane_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(surface_buoy_mission_node src/surface_buoy_mission_node.cpp)
target_compile_features(surface_buoy_mission_node PUBLIC cxx_std_17)
target_include_directories(surface_buoy_mission_node PRIVATE include)
target_link_libraries(surface_buoy_mission_node surface_lane_planner)
ament_target_dependencies(
  surface_buoy_mission_node
  auv_msg
  geometry_msgs
  mavros_msgs
  nav_msgs
  rclcpp
  std_msgs
)

install(
  TARGETS surface_lane_planner surface_buoy_mission_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_surface_buoy_mission test/test_surface_buoy_mission.cpp)
  target_include_directories(test_surface_buoy_mission PRIVATE include)
  target_link_libraries(test_surface_buoy_mission surface_lane_planner)
endif()

ament_package()
