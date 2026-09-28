cmake_minimum_required(VERSION 3.10.2)
project(realsense_ros2 VERSION 1.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 相机节点直接复用官方 realsense2_camera 的 realsense2_camera::RealSenseNodeFactory 组件
# （见 package.xml 的 exec_depend）；本包只编译按需录制组件。
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# --- recorder component: 服务触发的定时 rosbag 录制，话题列表由参数给出 ---
ament_auto_add_library(topic_recorder_component SHARED
  src/recorder/topic_recorder_component.cpp
)
target_include_directories(topic_recorder_component
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)
rclcpp_components_register_nodes(
  topic_recorder_component
  "realsense_ros2::TopicRecorderComponent"
)

# 联合录制协调节点：/record_all 同时触发 OAK 与 RealSense 录制
install(PROGRAMS
  scripts/record_all.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_auto_package(INSTALL_TO_SHARE launch config)
