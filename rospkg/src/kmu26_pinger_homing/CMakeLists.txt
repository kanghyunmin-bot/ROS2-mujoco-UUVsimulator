cmake_minimum_required(VERSION 3.8)
project(kmu26_pinger_homing)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(pinger_audio_estimator src/pinger_audio_estimator.cpp)
ament_target_dependencies(pinger_audio_estimator audio_common_msgs rclcpp std_msgs)

add_executable(pinger_frequency_selector src/pinger_frequency_selector.cpp)
ament_target_dependencies(pinger_frequency_selector audio_common_msgs rclcpp std_msgs)
target_link_libraries(pinger_frequency_selector Eigen3::Eigen)

add_executable(pinger_homing_2d_controller src/pinger_homing_2d_controller.cpp)
ament_target_dependencies(pinger_homing_2d_controller geometry_msgs mavros_msgs nav_msgs rclcpp sensor_msgs std_msgs)
target_include_directories(pinger_homing_2d_controller PRIVATE ${EIGEN3_INCLUDE_DIRS})

# Canonical C++ controller restored from the proven deployed Phase sequence.
# Keep the older 2-D prototype built for comparison, but do not make a launch
# path depend on it: the canonical controller owns the production contracts.
add_executable(pinger_homing_controller
  src/pinger_homing/pinger_homing_controller.cpp)
target_compile_features(pinger_homing_controller PUBLIC cxx_std_17)
ament_target_dependencies(
  pinger_homing_controller rclcpp geometry_msgs
  mavros_msgs nav_msgs sensor_msgs std_msgs)
target_link_libraries(pinger_homing_controller Eigen3::Eigen)

add_executable(rc_override_mux src/rc_override_mux.cpp)
target_compile_features(rc_override_mux PUBLIC cxx_std_17)
ament_target_dependencies(rc_override_mux rclcpp mavros_msgs std_msgs)

if(BUILD_TESTING)
  enable_testing()
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(
    NAME single_hydrophone_homing_math
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_single_hydrophone_homing_math.py)
  add_test(
    NAME pinger_homing_dry_run
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pinger_homing_dry_run.py
      --controller ${CMAKE_CURRENT_SOURCE_DIR}/src/pinger_homing/single_hydrophone_homing_controller.py)
  add_test(
    NAME pinger_homing_live_runtime
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pinger_homing_live_runtime.py
      --controller ${CMAKE_CURRENT_SOURCE_DIR}/src/pinger_homing/single_hydrophone_homing_controller.py)
  add_test(
    NAME rc_override_mux_runtime
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_rc_override_mux_runtime.py
      --mux $<TARGET_FILE:rc_override_mux>)
  add_test(
    NAME pinger_rc_pipeline_runtime
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pinger_rc_pipeline_runtime.py
      --controller ${CMAKE_CURRENT_SOURCE_DIR}/src/pinger_homing/single_hydrophone_homing_controller.py
      --mux $<TARGET_FILE:rc_override_mux>)
  add_test(
    NAME pinger_hydrophone_repository_boundary
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_repository_boundaries.py)
  add_test(
    NAME pinger_2d_homing_contract
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_2d_homing_contract.py)
  add_test(
    NAME pinger_real_phase_launch_contract
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_real_phase_launch_contract.py)
  add_test(
    NAME no_odom_xy_depthless_runtime
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_no_odom_xy_depthless_runtime.py
      --controller $<TARGET_FILE:pinger_homing_controller>)
  add_test(
    NAME motion_response_runtime
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_motion_response_runtime.py
      --controller $<TARGET_FILE:pinger_homing_controller>)
  add_test(
    NAME adaptive_phase_innovation_runtime
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_adaptive_phase_innovation_runtime.py
      --controller $<TARGET_FILE:pinger_homing_controller>)
  add_test(
    NAME frequency_selector_runtime
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_frequency_selector_runtime.py
      --selector $<TARGET_FILE:pinger_frequency_selector>)
  add_test(
    NAME cpp_odometry_mode_runtime
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_cpp_odometry_mode_runtime.py
      --controller $<TARGET_FILE:pinger_homing_controller>)
  add_test(
    NAME cpp_odometry_audio_bootstrap_runtime
    COMMAND ${Python3_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/test/test_cpp_odometry_mode_runtime.py
      --controller $<TARGET_FILE:pinger_homing_controller>
      --quality-only)
  set_tests_properties(single_hydrophone_homing_math PROPERTIES TIMEOUT 30)
  set_tests_properties(pinger_homing_dry_run PROPERTIES TIMEOUT 15)
  set_tests_properties(pinger_homing_live_runtime PROPERTIES TIMEOUT 15)
  set_tests_properties(rc_override_mux_runtime PROPERTIES TIMEOUT 15)
  set_tests_properties(pinger_rc_pipeline_runtime PROPERTIES TIMEOUT 15)
  set_tests_properties(pinger_hydrophone_repository_boundary PROPERTIES TIMEOUT 10)
  set_tests_properties(pinger_real_phase_launch_contract PROPERTIES TIMEOUT 10)
  set_tests_properties(no_odom_xy_depthless_runtime PROPERTIES TIMEOUT 10)
  set_tests_properties(motion_response_runtime PROPERTIES TIMEOUT 10)
  set_tests_properties(adaptive_phase_innovation_runtime PROPERTIES TIMEOUT 15)
  set_tests_properties(frequency_selector_runtime PROPERTIES TIMEOUT 10)
  set_tests_properties(cpp_odometry_mode_runtime PROPERTIES TIMEOUT 10)
  set_tests_properties(cpp_odometry_audio_bootstrap_runtime PROPERTIES TIMEOUT 10)
endif()

install(TARGETS
  pinger_audio_estimator
  pinger_frequency_selector
  pinger_homing_2d_controller
  pinger_homing_controller
  rc_override_mux
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  src/pinger_homing/single_hydrophone_homing_controller.py
  scripts/start_pinger_homing_test_tank.sh
  DESTINATION lib/${PROJECT_NAME})

# Keep the math module beside the controller so direct ROS execution can import it.
install(FILES
  src/pinger_homing/single_hydrophone_homing_math.py
  DESTINATION lib/${PROJECT_NAME})

install(FILES
  PINGER_HOMING_ARCHITECTURE_KO.md
  DESTINATION share/${PROJECT_NAME})

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY src/pinger_homing DESTINATION share/${PROJECT_NAME}/algorithms
  FILES_MATCHING PATTERN "*.md" PATTERN "__pycache__" EXCLUDE)

ament_package()
