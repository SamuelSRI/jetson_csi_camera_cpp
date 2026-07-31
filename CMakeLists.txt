cmake_minimum_required(VERSION 3.8)
project(jetson_csi_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(PkgConfig REQUIRED)

pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)

add_executable(csi_camera_node src/csi_camera_node.cpp)

ament_target_dependencies(csi_camera_node
  rclcpp
  sensor_msgs
)

target_link_libraries(csi_camera_node
  PkgConfig::GSTREAMER
)

install(TARGETS
  csi_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
