cmake_minimum_required(VERSION 3.10)
project(auv_imx219_camera)

include(CTest)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(image_transport REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

pkg_check_modules(
  GSTREAMER
  REQUIRED
  IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)

add_executable(imx219_camera_node src/imx219_camera_node.cpp)
target_compile_features(imx219_camera_node PUBLIC cxx_std_17)
ament_target_dependencies(imx219_camera_node image_transport rclcpp sensor_msgs)
target_link_libraries(imx219_camera_node PkgConfig::GSTREAMER)

install(TARGETS imx219_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES LICENSE README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  add_executable(test_capture_stamp_selector test/test_capture_stamp_selector.cpp)
  target_compile_features(test_capture_stamp_selector PUBLIC cxx_std_17)
  target_include_directories(test_capture_stamp_selector PRIVATE src)
  add_test(NAME test_capture_stamp_selector COMMAND test_capture_stamp_selector)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
