import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    pkg_dir = get_package_share_directory("oak_ros2")

    launch_args = [
        DeclareLaunchArgument("tf_prefix", default_value="oak"),
        DeclareLaunchArgument("camera_name", default_value="ov9782"),
        DeclareLaunchArgument("camera_type", default_value="color"),
        DeclareLaunchArgument("resolution", default_value="800p"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("oak_fw_uri", default_value=""),
        DeclareLaunchArgument("imu_config_path", default_value=os.path.join(pkg_dir, "config", "imu.yaml")),
        DeclareLaunchArgument("compressed", default_value="true"),
        DeclareLaunchArgument("imu_hz", default_value="200"),
        DeclareLaunchArgument("cam_board_sockets", default_value="[CAM_A, CAM_B, CAM_C, CAM_D]"),
        DeclareLaunchArgument("sync_threshold_ms", default_value="50"),
        DeclareLaunchArgument("image_queue_size", default_value="1"),
        DeclareLaunchArgument("imu_queue_size", default_value="50"),
        DeclareLaunchArgument("lazy_publisher", default_value="true"),
        DeclareLaunchArgument("manual_exposure_us", default_value="1000"),
        DeclareLaunchArgument("manual_iso", default_value="800"),
        DeclareLaunchArgument("exposure_compensation", default_value="0"),
        DeclareLaunchArgument("manual_wb_kelvin", default_value="5000"),
        DeclareLaunchArgument("brightness", default_value="0"),
        DeclareLaunchArgument("contrast", default_value="0"),
        DeclareLaunchArgument("saturation", default_value="0"),
        DeclareLaunchArgument("sharpness", default_value="1"),
        DeclareLaunchArgument("luma_denoise", default_value="1"),
        DeclareLaunchArgument("chroma_denoise", default_value="1"),
        DeclareLaunchArgument("manual_focus", default_value="-1"),
    ]

    container = ComposableNodeContainer(
        name="oak_foxglove_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container_isolated",  # 或使用 component_container / component_container_mt
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
                    {"compressed": LaunchConfiguration("compressed")},
                    {"imu_hz": LaunchConfiguration("imu_hz")},
                    {"cam_board_sockets": LaunchConfiguration("cam_board_sockets")},
                    {"sync_threshold_ms": LaunchConfiguration("sync_threshold_ms")},
                    {"image_queue_size": LaunchConfiguration("image_queue_size")},
                    {"imu_queue_size": LaunchConfiguration("imu_queue_size")},
                    {"lazy_publisher": LaunchConfiguration("lazy_publisher")},
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
