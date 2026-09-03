import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


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
        DeclareLaunchArgument("resolution", default_value="800p"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("oak_fw_uri", default_value=""),
        DeclareLaunchArgument("imu_config_path", default_value=os.path.join(pkg_dir, "config", "imu.yaml")),
        DeclareLaunchArgument("imu_hz", default_value="200"),
        DeclareLaunchArgument("cam_board_sockets", default_value="[CAM_A, CAM_B, CAM_C, CAM_D]"),
        DeclareLaunchArgument("sync_threshold_ms", default_value="50"),
        DeclareLaunchArgument("image_queue_size", default_value="1"),
        DeclareLaunchArgument("imu_queue_size", default_value="50"),
        DeclareLaunchArgument("manual_exposure_us", default_value="1000"),
        DeclareLaunchArgument("manual_iso", default_value="800"),
        DeclareLaunchArgument("exposure_compensation", default_value="0"),
        DeclareLaunchArgument("manual_wb_kelvin", default_value="3500"),
        DeclareLaunchArgument("brightness", default_value="0"),
        DeclareLaunchArgument("contrast", default_value="0"),
        DeclareLaunchArgument("saturation", default_value="0"),
        DeclareLaunchArgument("sharpness", default_value="1"),
        DeclareLaunchArgument("luma_denoise", default_value="1"),
        DeclareLaunchArgument("chroma_denoise", default_value="1"),
        DeclareLaunchArgument("manual_focus", default_value="-1"),
        # 录制相关参数
        DeclareLaunchArgument("bag_uri", default_value=default_bag_uri),
        DeclareLaunchArgument("storage_id", default_value="mcap"),
    ]

    # 与 compressed 发布时保持一致的话题名：
    #   图像 -> /CAM_x/image/compressed (sensor_msgs/CompressedImage)w
    #   IMU  -> /imu                    (sensor_msgs/Imu)
    record_topics = [
        "/imu",
        "/CAM_A/image/compressed",
        "/CAM_B/image/compressed",
        "/CAM_C/image/compressed",
        "/CAM_D/image/compressed",
    ]

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
                    {"compressed": True},
                    {"imu_hz": LaunchConfiguration("imu_hz")},
                    {"cam_board_sockets": LaunchConfiguration("cam_board_sockets")},
                    {"sync_threshold_ms": LaunchConfiguration("sync_threshold_ms")},
                    {"image_queue_size": LaunchConfiguration("image_queue_size")},
                    {"imu_queue_size": LaunchConfiguration("imu_queue_size")},
                    # 录制时必须实时产生消息，关闭懒发布
                    {"lazy_publisher": False},
                    {"manual_exposure_us": LaunchConfiguration("manual_exposure_us")},
                    {"manual_iso": LaunchConfiguration("manual_iso")},
                    {"exposure_compensation": LaunchConfiguration("exposure_compensation")},
                    {"manual_wb_kelvin": LaunchConfiguration("manual_wb_kelvin")},
                    {"brightness": LaunchConfiguration("brightness")},
                    {"contrast": LaunchConfiguration("contrast")},
                    {"saturation": LaunchConfiguration("saturation")},
                    {"sharpness": LaunchConfiguration("sharpness")},
                    {"luma_denoise": LaunchConfiguration("luma_denoise")},
                    {"chroma_denoise": LaunchConfiguration("chroma_denoise")},
                    {"manual_focus": LaunchConfiguration("manual_focus")},
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
                        "storage.uri": LaunchConfiguration("bag_uri"),
                        "storage.storage_id": LaunchConfiguration("storage_id"),
                        # 及时落盘，Ctrl+C 收尾更快、更不易在关停超时内丢数据
                        "storage.max_cache_size": 10485760,  # 10 MiB
                    }
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    return LaunchDescription([*launch_args, container])
