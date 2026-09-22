import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# 相机 ISP 参数集中在同目录的 oak_camera_params.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oak_camera_params import camera_launch_args, camera_parameters  # noqa: E402


def generate_launch_description():
    pkg_dir = get_package_share_directory("oak_ros2")

    launch_args = [
        DeclareLaunchArgument("tf_prefix", default_value="oak"),
        DeclareLaunchArgument("camera_name", default_value="ov9782"),
        DeclareLaunchArgument("camera_type", default_value="color"),
        DeclareLaunchArgument("resolution", default_value="400p"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("oak_fw_uri", default_value=""),
        DeclareLaunchArgument("imu_config_path", default_value=os.path.join(pkg_dir, "config", "imu.yaml")),
        DeclareLaunchArgument("imu_correction_enabled", default_value="true"),
        DeclareLaunchArgument("compressed", default_value="false"),
        DeclareLaunchArgument("imu_hz", default_value="200"),
        DeclareLaunchArgument("cam_board_sockets", default_value="[CAM_A, CAM_B, CAM_C, CAM_D]"),
        DeclareLaunchArgument("sync_threshold_ms", default_value="30"),
        DeclareLaunchArgument("image_queue_size", default_value="1"),
        DeclareLaunchArgument("imu_queue_size", default_value="50"),
        DeclareLaunchArgument("lazy_publisher", default_value="true"),
        *camera_launch_args(),
    ]

    container = ComposableNodeContainer(
        name="oak_foxglove_container",
        namespace="",
        package="rclcpp_components",
        # component_container_isolated 已废弃，改用统一入口 + 参数
        executable="component_container",
        arguments=["--executor-type", "single-threaded", "--isolated"],
        composable_node_descriptions=[
            # 1. OAK 相机与 IMU 组件
            ComposableNode(
                package="oak_ros2",
                plugin="oak_ffc_camera_imu::OakFfcCameraImuComponent",
                name="oak_ffc_camera_imu",
                parameters=[
                    {"tf_prefix": LaunchConfiguration("tf_prefix")},
                    {"camera_name": LaunchConfiguration("camera_name")},
                    {"camera_type": LaunchConfiguration("camera_type")},
                    {"resolution": LaunchConfiguration("resolution")},
                    {"fps": LaunchConfiguration("fps")},
                    {"oak_fw_uri": LaunchConfiguration("oak_fw_uri")},
                    {"imu_config_path": LaunchConfiguration("imu_config_path")},
                    {"imu_correction_enabled": LaunchConfiguration("imu_correction_enabled")},
                    {"compressed": LaunchConfiguration("compressed")},
                    {"imu_hz": LaunchConfiguration("imu_hz")},
                    {"cam_board_sockets": LaunchConfiguration("cam_board_sockets")},
                    {"sync_threshold_ms": LaunchConfiguration("sync_threshold_ms")},
                    {"image_queue_size": LaunchConfiguration("image_queue_size")},
                    {"imu_queue_size": LaunchConfiguration("imu_queue_size")},
                    {"lazy_publisher": LaunchConfiguration("lazy_publisher")},
                    *camera_parameters(),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # 2. Foxglove Bridge 组件
            ComposableNode(
                package="foxglove_bridge",
                plugin="foxglove_bridge::FoxgloveBridge",
                name="foxglove_bridge",
                parameters=[
                    # 可根据需要添加 foxglove_bridge 的参数配置，例如 port 等
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([*launch_args, container])
