cmake_minimum_required(VERSION 3.8)
project(hit25_auv_ros2)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(dvl_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

add_executable(cam2baselink_static_tf src/cam2baselink_static_tf.cpp)
ament_target_dependencies(cam2baselink_static_tf rclcpp geometry_msgs tf2 tf2_ros)

add_executable(tf_broadcaster src/tf_broadcaster.cpp)
ament_target_dependencies(tf_broadcaster rclcpp geometry_msgs tf2 tf2_ros)

add_executable(joy2mavros src/joy2mavros.cpp)
ament_target_dependencies(joy2mavros rclcpp sensor_msgs mavros_msgs)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME sim_mavros_time_contract
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_sim_mavros_time_contract.py
  )
  set_tests_properties(sim_mavros_time_contract PROPERTIES TIMEOUT 10)
  add_test(
    NAME dvl_frd_to_flu_contract
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_dvl_frd_to_flu_contract.py
  )
  set_tests_properties(dvl_frd_to_flu_contract PROPERTIES TIMEOUT 10)
endif()

add_executable(odom2mavros src/odom2mavros.cpp)
ament_target_dependencies(odom2mavros rclcpp nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

add_executable(vfr2atm_pressure src/vfr2atm_pressure.cpp)
ament_target_dependencies(vfr2atm_pressure rclcpp mavros_msgs sensor_msgs)

add_executable(dvl_to_twist_bridge src/dvl_to_twist_bridge.cpp)
ament_target_dependencies(dvl_to_twist_bridge rclcpp geometry_msgs dvl_msgs)

add_executable(dvl_position_to_odom_bridge src/dvl_position_to_odom_bridge.cpp)
ament_target_dependencies(
  dvl_position_to_odom_bridge
  rclcpp
  dvl_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)

add_executable(pressure_to_depth_pose src/pressure_to_depth_pose.cpp)
ament_target_dependencies(pressure_to_depth_pose rclcpp geometry_msgs sensor_msgs)

add_executable(localization_debug src/localization_debug.cpp)
ament_target_dependencies(localization_debug rclcpp geometry_msgs nav_msgs)

add_executable(guided_control src/guided_control.cpp)
ament_target_dependencies(guided_control rclcpp geometry_msgs tf2 mavros_msgs)

add_executable(buoy_position_control src/buoy_position_control.cpp)
ament_target_dependencies(
  buoy_position_control
  rclcpp
  geometry_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mavros_msgs
)

install(TARGETS
  cam2baselink_static_tf
  tf_broadcaster
  joy2mavros
  odom2mavros
  vfr2atm_pressure
  dvl_to_twist_bridge
  dvl_position_to_odom_bridge
  pressure_to_depth_pose
  localization_debug
  guided_control
  buoy_position_control
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/dvl_setup.sh
  scripts/dronecan2mavros_battery.py
  scripts/dronecan2mavros_battery_v2.py
  scripts/mavros_imu_rate_config.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
