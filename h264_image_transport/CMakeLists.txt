cmake_minimum_required(VERSION 3.5)

project(h264_image_transport)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Emulate colcon in CLion
if($ENV{CLION_IDE})
  message("Running inside CLion")
  set(h264_msgs_DIR
    "${PROJECT_SOURCE_DIR}/../../../install/h264_msgs/share/h264_msgs/cmake")
  find_package(fastrtps_cmake_module REQUIRED)
  set(FastRTPS_INCLUDE_DIR "/opt/ros/foxy/include")
  set(FastRTPS_LIBRARY_RELEASE "/opt/ros/foxy/lib/libfastrtps.so")
endif()

find_package(ament_cmake REQUIRED)
find_package(camera_calibration_parsers REQUIRED)
find_package(image_transport REQUIRED)
find_package(h264_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# Find ffmpeg libraries
find_package(PkgConfig)
pkg_check_modules(PC_AVDEVICE REQUIRED libavdevice>=58)
pkg_check_modules(PC_AVFORMAT REQUIRED libavformat>=58)
pkg_check_modules(PC_AVCODEC REQUIRED libavcodec>=58)
pkg_check_modules(PC_AVUTIL REQUIRED libavutil>=56)
pkg_check_modules(PC_SWSCALE REQUIRED libswscale>=5)

add_library(
  ${PROJECT_NAME} SHARED
  src/h264_subscriber.cpp
  src/manifest.cpp
)

include_directories(
  include
)

target_link_libraries(
  ${PROJECT_NAME}
  ${PC_AVDEVICE_LIBRARIES}
  ${PC_AVFORMAT_LIBRARIES}
  ${PC_AVCODEC_LIBRARIES}
  ${PC_AVUTIL_LIBRARIES}
  ${PC_SWSCALE_LIBRARIES}
)

ament_target_dependencies(
  ${PROJECT_NAME}
  h264_msgs
  image_transport
  rclcpp
  pluginlib
  sensor_msgs
)

add_executable(
  h264_cam_node
  src/h264_cam_node.cpp
)

ament_target_dependencies(
  h264_cam_node
  camera_calibration_parsers
  h264_msgs
  rclcpp
)

target_link_libraries(
  h264_cam_node
  ${PC_AVDEVICE_LIBRARIES}
  ${PC_AVFORMAT_LIBRARIES}
  ${PC_AVCODEC_LIBRARIES}
  ${PC_AVUTIL_LIBRARIES}
)

# This will load & run linters listed in package.xml
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  TARGETS ${PROJECT_NAME} h264_cam_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib                 # Shared libraries must be in lib
  RUNTIME DESTINATION lib/${PROJECT_NAME} # Node executables must be in lib/<pkg>
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

pluginlib_export_plugin_description_file(image_transport h264_plugins.xml)

ament_package()
