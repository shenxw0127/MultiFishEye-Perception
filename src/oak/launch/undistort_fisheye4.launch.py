import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

# OAK 驱动组件参数（分辨率/帧率/ISP 等）集中在同目录的 oak_camera_params.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oak_camera_params import ns_launch_arg, oak_launch_args, oak_parameters  # noqa: E402


def generate_launch_description():
    pkg_dir = get_package_share_directory("oak_ros2")
    camera_cfg_dir = os.path.join(pkg_dir, "config", "camera")

    # 本 launch 下游要实时消费 raw 图像，写死不暴露
    FIXED = {"compressed": False, "lazy_publisher": False}

    launch_args = [
        ns_launch_arg(),
        *oak_launch_args(FIXED),
    ]

    # 四路相机对应的 Kalibr camchain 标定文件
    cameras = [
        ("CAM_A", "cama-camchain.yaml"),
        ("CAM_B", "camb-camchain.yaml"),
        ("CAM_C", "camc-camchain.yaml"),
        ("CAM_D", "camd-camchain.yaml"),
    ]

    ns = LaunchConfiguration("ns")
    undistort_nodes = [
        ComposableNode(
            package="oak_ros2",
            plugin="oak_ffc_camera_imu::FisheyeUndistortComponent",
            name=f"undistort_{cam.lower()}",
            # 输出在 /<ns>/undistort/<cam>/image_rect
            namespace=[ns, "/undistort"],
            parameters=[
                {"camchain_path": os.path.join(camera_cfg_dir, camchain)},
                {"input_topic": ["/", ns, f"/{cam}/image"]},
                {"output_topic": f"{cam}/image_rect"},
                {"output_frame_id": [LaunchConfiguration("tf_prefix"), f"_{cam}_optical_frame"]},
            ],
            extra_arguments=[{"use_intra_process_comms": True}],
        )
        for cam, camchain in cameras
    ]

    container = ComposableNodeContainer(
        name="oak_undistort_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        arguments=["--executor-type", "single-threaded", "--isolated"],
        composable_node_descriptions=[
            # OAK 相机与 IMU 组件（压缩图像输出）
            ComposableNode(
                package="oak_ros2",
                plugin="oak_ffc_camera_imu::OakFfcCameraImuComponent",
                name="oak_ffc_camera_imu",
                namespace=ns,
                parameters=[
                    *oak_parameters(FIXED),
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            *undistort_nodes,
        ],
        output="screen",
    )

    return LaunchDescription([*launch_args, container])
