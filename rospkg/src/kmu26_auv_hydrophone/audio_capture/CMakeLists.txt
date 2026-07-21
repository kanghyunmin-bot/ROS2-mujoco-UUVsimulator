cmake_minimum_required(VERSION 3.10)
project(audio_capture)

# 1. 컴포넌트 빌드를 위한 필수 패키지들을 불러옵니다.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(audio_common_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_check_modules(GST1.0 gstreamer-1.0 REQUIRED)
pkg_check_modules(GST_APP gstreamer-app-1.0 REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread)

# 2. add_executable 대신 add_library(... SHARED)를 사용해 공유 라이브러리로 빌드합니다.
add_library(${PROJECT_NAME} SHARED
  src/audio_capture_node.cpp
  src/audio_doppler_homing_node.cpp
  src/audio_frequency_detector.cpp
  src/audio_phase_estimator.cpp
  src/snr_gradient_homing_node.cpp
  src/snr_gradient_homing_node_v2.cpp
  src/snr_gradient_homing_controller_node.cpp
  src/homing_controller_node.cpp
  src/simple_homing_controller_node.cpp
  src/fft_node.cpp
)

# 3. 단축키(ament_target_dependencies) 대신 수동으로 헤더 파일 경로를 지정합니다.
target_include_directories(${PROJECT_NAME}
  PRIVATE
    ${rclcpp_INCLUDE_DIRS}
    ${rclcpp_components_INCLUDE_DIRS}
    ${std_msgs_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${mavros_msgs_INCLUDE_DIRS}
    ${audio_common_msgs_INCLUDE_DIRS}
    ${diagnostic_updater_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${GST1.0_INCLUDE_DIRS}
    ${GST_APP_INCLUDE_DIRS}
)

# 4. 수동으로 필요한 라이브러리 부품들을 연결(Link)합니다.
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    ${rclcpp_LIBRARIES}
    ${rclcpp_components_LIBRARIES}
    ${std_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${mavros_msgs_LIBRARIES}
    ${audio_common_msgs_LIBRARIES}
    ${diagnostic_updater_LIBRARIES}
  PRIVATE
    ${GST1.0_LIBRARIES}
    ${GST_APP_LIBRARIES}
    ${Boost_LIBRARIES}
)

# 5. 이 라이브러리를 ROS 2 시스템에 정식 컴포넌트 플러그인으로 등록합니다.
# PLUGIN 뒤에는 C++ 코드에서 정의한 [네임스페이스::클래스명]이 들어갑니다.
# EXECUTABLE을 지정하면 컴포넌트를 단독 실행할 수 있는 일반 실행 파일(_node)도 자동으로 만들어집니다.
rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "audio_capture::AudioCaptureNode" 
  EXECUTABLE audio_capture_node 
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "audio_capture::AudioFrequencyDetectorNode"
  EXECUTABLE audio_frequency_detector
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "audio_capture::AudioDopplerHomingNode"
  EXECUTABLE audio_doppler_homing
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "audio_capture::AudioPhaseEstimatorNode"
  EXECUTABLE audio_phase_estimator
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "audio_capture::SnrGradientHomingNode"
  EXECUTABLE snr_gradient_homing
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "audio_capture::SnrGradientHomingNodeV2"
  EXECUTABLE snr_gradient_homing_v2
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "audio_capture::SnrGradientHomingControllerNode"
  EXECUTABLE snr_gradient_homing_controller
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "audio_capture::HomingControllerNode"
  EXECUTABLE homing_controller
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "audio_capture::SimpleHomingControllerNode"
  EXECUTABLE simple_homing_controller
)

rclcpp_components_register_node(
  ${PROJECT_NAME}
  PLUGIN "audio_capture::AudioFftNode"
  EXECUTABLE fft_node
)

# 6. 생성된 컴포넌트 라이브러리와 단독 실행 파일을 모두 알맞은 폴더에 설치합니다.
install(TARGETS 
  ${PROJECT_NAME} 
  audio_capture_node
  audio_doppler_homing
  audio_frequency_detector
  audio_phase_estimator
  snr_gradient_homing
  snr_gradient_homing_v2
  snr_gradient_homing_controller
  homing_controller
  simple_homing_controller
  fft_node
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/snr_map_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_libraries(
  ${PROJECT_NAME}
)



ament_package()
