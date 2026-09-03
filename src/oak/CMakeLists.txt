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

# --- camera_imu component: 同步 OAK 相机图像 + IMU ---
set(camera_imu_sources
  src/camera_imu/component.cpp
  src/camera_imu/camera_controls.cpp
  src/camera_imu/config.cpp
  src/camera_imu/image_publisher.cpp
  src/camera_imu/imu_correction.cpp
  src/camera_imu/imu_publisher.cpp
  src/camera_imu/pipeline_builder.cpp
)

ament_auto_add_library(camera_imu_component SHARED ${camera_imu_sources})
target_include_directories(camera_imu_component
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(camera_imu_component
  depthai::core
  yaml-cpp
)
rclcpp_components_register_nodes(
  camera_imu_component
  "oak_ffc_camera_imu::OakFfcCameraImuComponent"
)

# --- mono_converter component: 彩色/压缩图像转 mono8 ---
ament_auto_add_library(mono_converter_component SHARED
  src/mono_converter/mono_image_converter_component.cpp
)
target_include_directories(mono_converter_component
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
rclcpp_components_register_nodes(
  mono_converter_component
  "oak_ffc_camera_imu::MonoImageConverterComponent"
)

# --- undistort component: 鱼眼去畸变 (equidistant -> 针孔) ---
ament_auto_add_library(undistort_component SHARED
  src/undistort/undistort_component.cpp
)
target_include_directories(undistort_component
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
target_link_libraries(undistort_component
  yaml-cpp
)
rclcpp_components_register_nodes(
  undistort_component
  "oak_ffc_camera_imu::FisheyeUndistortComponent"
)

ament_auto_package(INSTALL_TO_SHARE launch config)
