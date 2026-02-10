cmake_minimum_required(VERSION 3.8)
project(hit25_auv)

# ROS 2 build system
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)

# ----------------------------------------------------------------------------
# Interface generation (ROS 2)
# ----------------------------------------------------------------------------
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/YawBias.msg"
  "msg/AuvSetpoint.msg"
  "msg/MissionStatus.msg"
  "msg/YawControlDebug.msg"
)

ament_export_dependencies(rosidl_default_runtime)

# ----------------------------------------------------------------------------
# C++ nodes (ROS 2)
# ----------------------------------------------------------------------------
set(_hit25_cpp_deps
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  mavros_msgs
)

add_executable(joy2mavros src/joy2mavros_node.cpp)
add_executable(odom2mavros src/odom2mavros_node.cpp)
add_executable(cam2baselink_static_tf src/cam2baselink_static_tf_node.cpp)
add_executable(guided_setposition_node src/guided_setposition_node.cpp)
add_executable(vfr2atm_pressure src/vfr2atm_pressure_node.cpp)
add_executable(guided_setpoint_yaw src/guided_setpoint_yaw_node.cpp)
add_executable(guided_control src/guided_control_node.cpp)

ament_target_dependencies(joy2mavros ${_hit25_cpp_deps})
ament_target_dependencies(odom2mavros ${_hit25_cpp_deps})
ament_target_dependencies(cam2baselink_static_tf ${_hit25_cpp_deps})
ament_target_dependencies(guided_setposition_node ${_hit25_cpp_deps})
ament_target_dependencies(vfr2atm_pressure ${_hit25_cpp_deps})
ament_target_dependencies(guided_setpoint_yaw ${_hit25_cpp_deps})
ament_target_dependencies(guided_control ${_hit25_cpp_deps})

install(TARGETS
  joy2mavros
  odom2mavros
  cam2baselink_static_tf
  guided_setposition_node
  vfr2atm_pressure
  guided_setpoint_yaw
  guided_control
  DESTINATION lib/${PROJECT_NAME}
)

# ----------------------------------------------------------------------------
# Python package (ROS 2)
# ----------------------------------------------------------------------------
# Install Python modules from scripts/ as the hit25_auv package. We avoid
# ament_python_install_package because rosidl also generates Python artifacts.
if(NOT PYTHON_INSTALL_DIR)
  message(FATAL_ERROR "PYTHON_INSTALL_DIR is not set. Ensure ament_cmake_python is found.")
endif()

install(DIRECTORY scripts/
  DESTINATION ${PYTHON_INSTALL_DIR}/${PROJECT_NAME}
  FILES_MATCHING
  PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

set(_hit25_py_execs
  "main_node:scripts/main_node.py"
  "rov_tf_broadcaster:scripts/rov_tf_broadcaster.py"
  "ref_point_marker_node:scripts/ref_point_marker_node.py"
  "setpoint_cli:scripts/setpoint_cli.py"
  "controller:scripts/controller.py"
  "buoy_mission:scripts/buoy_mission.py"
  "gate_mission:scripts/gate_mission.py"
  "line_qr_mission:scripts/line_qr_mission.py"
  "hydrophone_mission:scripts/hydrophone_mission.py"
  "return_mission:scripts/return_mission.py"
  "dronecan2mavros_battery:scripts/dronecan2mavros_battery.py"
  "battery_gui:scripts/battery_gui.py"
  "auv_state_gui:scripts/auv_state_gui.py"
)

foreach(_pair IN LISTS _hit25_py_execs)
  string(REPLACE ":" ";" _parts "${_pair}")
  list(GET _parts 0 _name)
  list(GET _parts 1 _path)
  install(PROGRAMS ${_path}
    DESTINATION lib/${PROJECT_NAME}
    RENAME ${_name}
  )
endforeach()

# ----------------------------------------------------------------------------
# Install non-code resources
# ----------------------------------------------------------------------------
# Install non-code resources (guard optional dirs)
set(_hit25_res_dirs meshes rviz urdf)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/config")
  list(APPEND _hit25_res_dirs config)
endif()
install(DIRECTORY ${_hit25_res_dirs}
  DESTINATION share/${PROJECT_NAME}
)

# Keep legacy ROS 1 sources for reference (if present).
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/ros1_legacy")
  install(DIRECTORY ros1_legacy
    DESTINATION share/${PROJECT_NAME}
  )
endif()

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Install headers (for future node porting)
install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

ament_package()
