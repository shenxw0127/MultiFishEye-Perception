import os
import sys
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterValue

# 相机 ISP 参数集中在同目录的 oak_camera_params.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oak_camera_params import camera_launch_args, camera_parameters  # noqa: E402


def unique_bag_uri(base):
    """若目标目录已存在，则追加 _N（N 从 1 开始）直到不冲突。"""
    if not os.path.exists(base):
        return base
    n = 1
    while os.path.exists(f"{base}_{n}"):
        n += 1
    return f"{base}_{n}"


def generate_launch_description():
    pkg_dir = get_package_share_directory("oak_ros2")

    default_bag_uri = os.path.join(
        os.path.expanduser("~"),
        "rosbags",
        "fisheye4_" + datetime.now().strftime("%Y%m%d_%H%M%S"),
    )

    launch_args = [
        DeclareLaunchArgument("tf_prefix", default_value="oak"),
        DeclareLaunchArgument("camera_name", default_value="ov9782"),
        DeclareLaunchArgument("camera_type", default_value="color"),
        DeclareLaunchArgument("resolution", default_value="400p"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("oak_fw_uri", default_value=""),
        DeclareLaunchArgument("imu_config_path", default_value=os.path.join(pkg_dir, "config", "imu.yaml")),
        # 录制默认关闭 IMU 校正，录到的是原始数据，可直接用于椭球标定。
        # 录制正常数据（已标定）时显式传 imu_correction_enabled:=true。
        DeclareLaunchArgument("imu_correction_enabled", default_value="true"),
        DeclareLaunchArgument("imu_hz", default_value="200"),
        DeclareLaunchArgument("cam_board_sockets", default_value="[CAM_A, CAM_B, CAM_C, CAM_D]"),
        DeclareLaunchArgument("sync_threshold_ms", default_value="30"),
        DeclareLaunchArgument("image_queue_size", default_value="1"),
        DeclareLaunchArgument("imu_queue_size", default_value="50"),
        *camera_launch_args(),
        # 录制相关参数
        DeclareLaunchArgument("bag_uri", default_value=default_bag_uri),
        DeclareLaunchArgument("storage_id", default_value="mcap"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
    ]

    # 与 compressed 发布时保持一致的话题名：
    #   图像 -> /CAM_x/image/compressed (sensor_msgs/CompressedImage)w
    #   IMU  -> /imu                    (sensor_msgs/Imu)
    record_topics = [
        "/imu",
        "/CAM_A/image",
        "/CAM_B/image",
        "/CAM_C/image",
        "/CAM_D/image",
    ]

    def launch_setup(context, *_args, **_kwargs):
        # 解析 bag_uri 实际取值（用户传入或默认），若目录已存在则追加 _N
        bag_uri = unique_bag_uri(LaunchConfiguration("bag_uri").perform(context))
        print(f"[record] 录制目录: {bag_uri}")

        container = ComposableNodeContainer(
            name="oak_record_container",
            namespace="",
            package="rclcpp_components",
            # component_container_isolated 已废弃，改用统一入口 + 参数
            executable="component_container",
            arguments=["--executor-type", "single-threaded", "--isolated"],
            # 给 recorder 足够时间在 Ctrl+C 后 finalize bag 文件，避免被过早 SIGKILL
            sigterm_timeout="30",
            sigkill_timeout="30",
            composable_node_descriptions=[
                # 1. OAK 相机与 IMU 组件（压缩图像输出）
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
                        {"compressed": False},
                        {"imu_hz": LaunchConfiguration("imu_hz")},
                        {"cam_board_sockets": LaunchConfiguration("cam_board_sockets")},
                        {"sync_threshold_ms": LaunchConfiguration("sync_threshold_ms")},
                        {"image_queue_size": LaunchConfiguration("image_queue_size")},
                        {"imu_queue_size": LaunchConfiguration("imu_queue_size")},
                        # 录制时必须实时产生消息，关闭懒发布
                        {"lazy_publisher": False},
                        *camera_parameters(),
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                # 2. rosbag2 录制组件（与相机组件同进程）
                ComposableNode(
                    package="rosbag2_transport",
                    plugin="rosbag2_transport::Recorder",
                    name="rosbag2_recorder",
                    parameters=[
                        {
                            "record.topics": record_topics,
                            "record.all_topics": False,
                            "record.all_services": False,
                            "record.all_actions": False,
                            "record.disable_keyboard_controls": True,
                            "storage.uri": bag_uri,
                            "storage.storage_id": LaunchConfiguration("storage_id"),
                            # 及时落盘，Ctrl+C 收尾更快、更不易在关停超时内丢数据
                            "storage.max_cache_size": 10485760,  # 10 MiB
                        }
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
                # 3. Foxglove Bridge 组件（实时预览录制中的话题）
                ComposableNode(
                    package="foxglove_bridge",
                    plugin="foxglove_bridge::FoxgloveBridge",
                    name="foxglove_bridge",
                    parameters=[
                        # port 是整型参数，需显式声明类型，否则字符串会解析失败
                        {"port": ParameterValue(LaunchConfiguration("foxglove_port"), value_type=int)},
                    ],
                    extra_arguments=[{"use_intra_process_comms": True}],
                ),
            ],
            output="screen",
        )

        return [container]

    return LaunchDescription([*launch_args, OpaqueFunction(function=launch_setup)])
