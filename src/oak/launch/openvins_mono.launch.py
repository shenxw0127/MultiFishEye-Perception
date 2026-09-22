import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

# 相机 ISP 参数集中在同目录的 oak_camera_params.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oak_camera_params import camera_launch_args, camera_parameters  # noqa: E402


# Mono diagnostic launch: single camera (CAM_A) + IMU + OpenVINS.
# Used to isolate whether VIO divergence comes from multi-camera extrinsic
# coupling or from the IMU / single-camera calibration.
#
# The estimator config lives in the source tree (not the installed share dir)
# so it can be edited without rebuilding:
#   src/open_vins/config/fisheye4_mono/estimator_config.yaml
def generate_launch_description():
    oak_pkg_dir = get_package_share_directory("oak_ros2")
    # Read estimator config from the source tree so it can be edited without a rebuild.
    # oak_pkg_dir = <ws>/install/oak_ros2/share/oak_ros2 -> go up 4 levels to <ws>.
    ws_root = os.path.abspath(os.path.join(oak_pkg_dir, "..", "..", "..", ".."))
    default_config = os.path.join(
        ws_root, "src", "open_vins", "config", "fisheye4_mono", "estimator_config.yaml",
    )

    launch_args = [
        DeclareLaunchArgument("tf_prefix", default_value="oak"),
        DeclareLaunchArgument("camera_name", default_value="ov9782"),
        DeclareLaunchArgument("camera_type", default_value="color"),
        DeclareLaunchArgument("resolution", default_value="800p"),
        DeclareLaunchArgument("fps", default_value="30"),
        DeclareLaunchArgument("oak_fw_uri", default_value=""),
        DeclareLaunchArgument("imu_config_path", default_value=os.path.join(oak_pkg_dir, "config", "imu.yaml")),
        DeclareLaunchArgument(
            "imu_correction_enabled",
            default_value="true",
            description="apply accel bias/scale + gyro bias correction (set false to publish raw IMU)",
        ),
        DeclareLaunchArgument("imu_hz", default_value="200"),
        DeclareLaunchArgument("cam_board_sockets", default_value="[CAM_A, CAM_B, CAM_C, CAM_D]"),
        DeclareLaunchArgument("sync_threshold_ms", default_value="30"),
        DeclareLaunchArgument("image_queue_size", default_value="1"),
        DeclareLaunchArgument("imu_queue_size", default_value="50"),
        *camera_launch_args(),
        DeclareLaunchArgument("config_path", default_value=default_config),
        DeclareLaunchArgument("save_total_state", default_value="false"),
        DeclareLaunchArgument("filepath_est", default_value="/tmp/openvins_mono/openvins_estimate.txt"),
        DeclareLaunchArgument("filepath_std", default_value="/tmp/openvins_mono/openvins_estimate_std.txt"),
        DeclareLaunchArgument("filepath_gt", default_value="/tmp/openvins_mono/openvins_groundtruth.txt"),
    ]

    oak_container = ComposableNodeContainer(
        name="oak_openvins_container",
        namespace="",
        package="rclcpp_components",
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
                    {"imu_correction_enabled": LaunchConfiguration("imu_correction_enabled")},
                    {"compressed": False},
                    {"imu_hz": LaunchConfiguration("imu_hz")},
                    {"cam_board_sockets": LaunchConfiguration("cam_board_sockets")},
                    {"sync_threshold_ms": LaunchConfiguration("sync_threshold_ms")},
                    {"image_queue_size": LaunchConfiguration("image_queue_size")},
                    {"imu_queue_size": LaunchConfiguration("imu_queue_size")},
                    {"lazy_publisher": False},
                    *camera_parameters(),
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
        ],
        output="screen",
    )

    openvins_node = Node(
        package="ov_msckf",
        executable="run_subscribe_msckf",
        namespace="ov_msckf",
        output="screen",
        parameters=[
            {"config_path": LaunchConfiguration("config_path")},
            {"verbosity": "INFO"},
            {"use_stereo": False},
            {"max_cameras": 1},
            {"save_total_state": LaunchConfiguration("save_total_state")},
            {"filepath_est": LaunchConfiguration("filepath_est")},
            {"filepath_std": LaunchConfiguration("filepath_std")},
            {"filepath_gt": LaunchConfiguration("filepath_gt")},
        ],
    )

    return LaunchDescription([*launch_args, oak_container, openvins_node])
