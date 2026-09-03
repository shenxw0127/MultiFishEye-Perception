import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterValue


def generate_launch_description():
    pkg_dir = get_package_share_directory("oak_ros2")
    camera_cfg_dir = os.path.join(pkg_dir, "config", "camera")

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
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
    ]

    # 四路相机对应的 Kalibr camchain 标定文件
    cameras = [
        ("CAM_A", "cama-camchain.yaml"),
        ("CAM_B", "camb-camchain.yaml"),
        ("CAM_C", "camc-camchain.yaml"),
        ("CAM_D", "camd-camchain.yaml"),
    ]

    undistort_nodes = [
        ComposableNode(
            package="oak_ros2",
            plugin="oak_ffc_camera_imu::FisheyeUndistortComponent",
            name=f"undistort_{cam.lower()}",
            namespace="undistort",
            parameters=[
                {"camchain_path": os.path.join(camera_cfg_dir, camchain)},
                {"input_topic": f"/{cam}/image"},
                {"output_topic": f"{cam}/image_rect"},
                {"output_frame_id": f"oak_{cam}_optical_frame"},
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        for cam, camchain in cameras
    ]

    container = ComposableNodeContainer(
        name="oak_foxglove_undistort_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        arguments=["--executor-type", "single-threaded", "--isolated"],
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
                    # raw 模式：进程内零拷贝传给去畸变组件，无需解码
                    {"compressed": False},
                    {"imu_hz": LaunchConfiguration("imu_hz")},
                    {"cam_board_sockets": LaunchConfiguration("cam_board_sockets")},
                    {"sync_threshold_ms": LaunchConfiguration("sync_threshold_ms")},
                    {"image_queue_size": LaunchConfiguration("image_queue_size")},
                    {"imu_queue_size": LaunchConfiguration("imu_queue_size")},
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
            # 2. 四路鱼眼去畸变组件
            *undistort_nodes,
            # 3. Foxglove Bridge 组件
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

    return LaunchDescription([*launch_args, container])
