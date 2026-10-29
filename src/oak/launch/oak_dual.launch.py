"""两台 OAK 同时运行：同一个 container 里每台一个驱动 + 一个录制组件，按 MxId 各绑一台，话题按命名空间分开。

    ros2 launch oak_ros2 oak_dual.launch.py
    ros2 launch oak_ros2 oak_dual.launch.py oak1_ns:=front oak2_ns:=rear fps:=15

oak1 = 双目 AR0234（CAM_A/CAM_B），oak2 = 四目 OV9782。

话题：/<oak1_ns>/CAM_A/image、/<oak1_ns>/imu …（默认 /oak1/...、/oak2/...）；frame_id 前缀同命名空间。
录制：ros2 service call /oak1/record oak_interfaces/srv/Record "{duration_s: 30}"，
bag 名为 <bag_prefix>_<ns>_<时间戳>，两台各录各的。
设备相关参数（命名空间、device_id、camera_name、插座）按 oak1_* / oak2_* 分别给，其余参数（分辨率、
曝光、录制设置等）两台共用。fps_display 只给 oak1 开，两个实例同时刷同一个终端会互相覆盖。
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterValue

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oak_camera_params import (  # noqa: E402
    oak_launch_args,
    oak_parameters,
    recorder_launch_args,
    recorder_parameters,
)

# camera_name 决定帧同步方式（见 pipeline_builder.cpp），必须和 sensor 一致：AR0234 用 OV9782 的
# 同步方式时一组同步帧都出不来，驱动会反复重连。sockets 写 ALL 则按实际插着的相机自动选路。
DEVICES = {
    "oak1": {"ns": "oak1", "device_id": "19443010E1BD652E00", "camera_name": "ar0234",
             "sockets": "[CAM_A, CAM_B]", "fps_display": True},
    "oak2": {"ns": "oak2", "device_id": "19443010319F502E00", "camera_name": "ov9782",
             "sockets": "[CAM_A, CAM_B, CAM_C, CAM_D]", "fps_display": False},
}

COMMON_FIXED = {"compressed": False}


def device_arg(name, key):
    return LaunchConfiguration(f"{name}_{key}")


def device_fixed(name, device):
    return {
        **COMMON_FIXED,
        "device_id": device_arg(name, "device_id"),
        "camera_name": device_arg(name, "camera_name"),
        "cam_board_sockets": device_arg(name, "sockets"),
        "tf_prefix": device_arg(name, "ns"),
        "fps_display": device["fps_display"],
    }


def device_nodes(name, device):
    ns = device_arg(name, "ns")
    return [
        ComposableNode(
            package="oak_ros2",
            plugin="oak_ffc_camera_imu::OakFfcCameraImuComponent",
            name="oak_ffc_camera_imu",
            namespace=ns,
            parameters=oak_parameters(device_fixed(name, device)),
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
        ComposableNode(
            package="oak_ros2",
            plugin="oak_ffc_camera_imu::BagRecorderComponent",
            name="oak_bag_recorder",
            namespace=ns,
            parameters=recorder_parameters(
                cam_board_sockets=device_arg(name, "sockets"),
                bag_prefix=[LaunchConfiguration("bag_prefix"), "_", ns],
            ),
            extra_arguments=[{"use_intra_process_comms": True}],
        ),
    ]


def generate_launch_description():
    device_args = [
        DeclareLaunchArgument(f"{name}_{key}", default_value=device[key])
        for name, device in DEVICES.items()
        for key in ("ns", "device_id", "camera_name", "sockets")
    ]
    # 设备相关的参数由上面的 oak1_* / oak2_* 提供，不再声明成共用参数
    launch_args = [
        *device_args,
        *oak_launch_args(device_fixed("oak1", DEVICES["oak1"]), resolution="400p"),
        *recorder_launch_args(),
        DeclareLaunchArgument("foxglove", default_value="false"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
    ]

    container = ComposableNodeContainer(
        name="oak_dual_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        arguments=["--executor-type", "single-threaded", "--isolated"],
        # 给录制组件足够时间在 Ctrl+C 后排空编码队列、关闭 bag，避免被过早 SIGKILL
        sigterm_timeout="30",
        sigkill_timeout="30",
        composable_node_descriptions=[
            *(node for name, device in DEVICES.items() for node in device_nodes(name, device)),
            ComposableNode(
                package="foxglove_bridge",
                plugin="foxglove_bridge::FoxgloveBridge",
                name="foxglove_bridge",
                parameters=[{"port": ParameterValue(LaunchConfiguration("foxglove_port"), value_type=int)}],
                extra_arguments=[{"use_intra_process_comms": True}],
                condition=IfCondition(LaunchConfiguration("foxglove")),
            ),
        ],
        output="screen",
    )

    return LaunchDescription([*launch_args, container])
