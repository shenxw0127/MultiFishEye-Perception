"""机载端主 launch：OAK 驱动（热插拔、400p raw）+ 按需定时录制 + 可选 foxglove。

    ros2 launch oak_ros2 oak.launch.py                 # 默认不开 foxglove
    ros2 launch oak_ros2 oak.launch.py foxglove:=true  # 地面调试时实时看图
    ros2 launch oak_ros2 oak.launch.py ns:=front       # 话题改到 /front/...

话题在 /<ns> 下（默认 ns=oak）：/oak/CAM_A/image、/oak/imu、/oak/isp_meta。

录制：
    ros2 service call /oak/record oak_interfaces/srv/Record "{duration_s: 30}"
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterValue

# OAK 驱动组件和录制组件的参数集中在同目录的 oak_camera_params.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from oak_camera_params import (  # noqa: E402
    ns_launch_arg,
    oak_launch_args,
    oak_parameters,
    recorder_launch_args,
    recorder_parameters
)


def generate_launch_description():
    # 机载端发布未去畸变的 raw 图像（bgr8），压缩只在录制时由主机线程做
    FIXED = {"compressed": False}

    launch_args = [
        ns_launch_arg(),
        *oak_launch_args(FIXED, resolution="400p"),
        *recorder_launch_args(),
        DeclareLaunchArgument("foxglove", default_value="true"),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
    ]

    container = ComposableNodeContainer(
        name="oak_container",
        namespace="",
        package="rclcpp_components",
        # component_container_isolated 已废弃，改用统一入口 + 参数
        executable="component_container",
        arguments=["--executor-type", "single-threaded", "--isolated"],
        # 给录制组件足够时间在 Ctrl+C 后排空编码队列、关闭 bag，避免被过早 SIGKILL
        sigterm_timeout="30",
        sigkill_timeout="30",
        composable_node_descriptions=[
            # 1. OAK 相机与 IMU 组件：后台自动连接/重连，相机不在时照常运行
            ComposableNode(
                package="oak_ros2",
                plugin="oak_ffc_camera_imu::OakFfcCameraImuComponent",
                name="oak_ffc_camera_imu",
                namespace=LaunchConfiguration("ns"),
                parameters=oak_parameters(FIXED),
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # 2. 录制组件：/<ns>/record 触发，只在录制期间订阅 raw 图像（进程内零拷贝）
            ComposableNode(
                package="oak_ros2",
                plugin="oak_ffc_camera_imu::BagRecorderComponent",
                name="oak_bag_recorder",
                namespace=LaunchConfiguration("ns"),
                parameters=recorder_parameters(),
                extra_arguments=[{"use_intra_process_comms": True}],
            ),
            # 3. Foxglove Bridge（可选）
            ComposableNode(
                package="foxglove_bridge",
                plugin="foxglove_bridge::FoxgloveBridge",
                name="foxglove_bridge",
                parameters=[
                    # port 是整型参数，需显式声明类型，否则字符串会解析失败
                    {"port": ParameterValue(LaunchConfiguration("foxglove_port"), value_type=int)},
                ],
                extra_arguments=[{"use_intra_process_comms": True}],
                condition=IfCondition(LaunchConfiguration("foxglove")),
            ),
        ],
        output="screen"
    )

    return LaunchDescription([*launch_args, container])
