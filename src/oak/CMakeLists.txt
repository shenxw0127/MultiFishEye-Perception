cmake_minimum_required(VERSION 3.10.2)
project(oak_ros2 VERSION 1.1.0 LANGUAGES CXX C)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

if(POLICY CMP0057)
  cmake_policy(SET CMP0057 NEW)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
find_package(yaml-cpp REQUIRED)

set(component_sources
  src/oak_ffc_camera_imu/component.cpp
  src/oak_ffc_camera_imu/camera_controls.cpp
  src/oak_ffc_camera_imu/config.cpp
  src/oak_ffc_camera_imu/image_publisher.cpp
  src/oak_ffc_camera_imu/imu_correction.cpp
  src/oak_ffc_camera_imu/imu_publisher.cpp
  src/oak_ffc_camera_imu/pipeline_builder.cpp
)

ament_auto_add_library(oak_ffc_camera_imu_component SHARED ${component_sources})
target_include_directories(oak_ffc_camera_imu_component
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(oak_ffc_camera_imu_component
  depthai::core
  yaml-cpp
)
rclcpp_components_register_nodes(
  oak_ffc_camera_imu_component
  "oak_ffc_camera_imu::OakFfcCameraImuComponent"
)

ament_auto_add_executable(oak_ffc_sync src/main.cpp)
target_link_libraries(oak_ffc_sync oak_ffc_camera_imu_component)

ament_auto_package(INSTALL_TO_SHARE launch config)
