import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    oak_pkg_dir = get_package_share_directory("oak_ros2")
    ov_pkg_dir = get_package_share_directory("ov_msckf")
    config_path = os.path.join(ov_pkg_dir, "config", "fisheye4", "estimator_config.yaml")

    launch_args = [
        DeclareLaunchArgument("tf_prefix", default_value="oak"),
        DeclareLaunchArgument("camera_name", default_value="ov9782"),
        DeclareLaunchArgument("camera_type", default_value="color"),
        DeclareLaunchArgument("resolution", default_value="800p"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("oak_fw_uri", default_value=""),
        DeclareLaunchArgument("imu_config_path", default_value=os.path.join(oak_pkg_dir, "config", "imu.yaml")),
        DeclareLaunchArgument("imu_hz", default_value="200"),
        DeclareLaunchArgument("cam_board_sockets", default_value="[CAM_A, CAM_B, CAM_C, CAM_D]"),
        DeclareLaunchArgument("sync_threshold_ms", default_value="50"),
        DeclareLaunchArgument("image_queue_size", default_value="1"),
        DeclareLaunchArgument("imu_queue_size", default_value="50"),
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
        DeclareLaunchArgument("save_total_state", default_value="false"),
        DeclareLaunchArgument("filepath_est", default_value="/tmp/openvins_fisheye4/openvins_estimate.txt"),
        DeclareLaunchArgument("filepath_std", default_value="/tmp/openvins_fisheye4/openvins_estimate_std.txt"),
        DeclareLaunchArgument("filepath_gt", default_value="/tmp/openvins_fisheye4/openvins_groundtruth.txt"),
    ]

    oak_container = ComposableNodeContainer(
        name="oak_openvins_container",
        namespace="",
        package="rclcpp_components",
        # component_container_isolated 已废弃，改用统一入口 + 参数
        executable="component_container",
        arguments=["--executor-type", "single-threaded", "--isolated"],
        composable_node_descriptions=[
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
            ComposableNode(
                package="oak_ros2",
                plugin="oak_ffc_camera_imu::MonoImageConverterComponent",
                name="mono_cam_a",
                namespace="openvins",
                parameters=[
                    {"input_topic": "/CAM_A/image"},
                    {"output_topic": "CAM_A/image_mono"},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="oak_ros2",
                plugin="oak_ffc_camera_imu::MonoImageConverterComponent",
                name="mono_cam_b",
                namespace="openvins",
                parameters=[
                    {"input_topic": "/CAM_B/image"},
                    {"output_topic": "CAM_B/image_mono"},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="oak_ros2",
                plugin="oak_ffc_camera_imu::MonoImageConverterComponent",
                name="mono_cam_c",
                namespace="openvins",
                parameters=[
                    {"input_topic": "/CAM_C/image"},
                    {"output_topic": "CAM_C/image_mono"},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            ComposableNode(
                package="oak_ros2",
                plugin="oak_ffc_camera_imu::MonoImageConverterComponent",
                name="mono_cam_d",
                namespace="openvins",
                parameters=[
                    {"input_topic": "/CAM_D/image"},
                    {"output_topic": "CAM_D/image_mono"},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
        ],
        output="screen",
    )

    openvins_node = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        namespace="ov_msckf",
        output="screen",
        parameters=[
            {"config_path": config_path},
            {"verbosity": "INFO"},
            {"use_stereo": False},
            {"max_cameras": 4},
            {"save_total_state": LaunchConfiguration("save_total_state")},
            {"filepath_est": LaunchConfiguration("filepath_est")},
            {"filepath_std": LaunchConfiguration("filepath_std")},
            {"filepath_gt": LaunchConfiguration("filepath_gt")},
        ],
    )

    return LaunchDescription([*launch_args, oak_container, openvins_node])
