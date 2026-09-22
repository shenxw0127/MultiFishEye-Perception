cmake_minimum_required(VERSION 3.10.2)
project(realsense_ros2 VERSION 1.0.0 LANGUAGES NONE)

# 本包只提供 launch：相机节点直接复用官方 realsense2_camera 的
# realsense2_camera::RealSenseNodeFactory 组件（见 package.xml 的 exec_depend），
# 因此不需要 ament_cmake_auto，也没有任何编译目标。
find_package(ament_cmake REQUIRED)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
