"""OAK 四目鱼眼 + RealSense D435i 联合录制（无硬件同步，靠时间戳软同步）。

    ros2 launch realsense_ros2 record_all.launch.py
    ros2 launch realsense_ros2 record_all.launch.py oak_record_hz:=4.0 bag_root:=/data/bags

录制（两台相机同时开录，bag 放进同一个 session 目录）：
    ros2 service call /record_all oak_interfaces/srv/Record "{duration_s: 30}"
    -> <bag_root>/session_<时间戳>/oak  与  <bag_root>/session_<时间戳>/d435i
单独录某一台照旧：/oak/record、/d435i/record。

进程结构：两台相机各一个容器（oak.launch.py、recorder.launch.py 原样 include），互不影响：
一台拔插 / 崩溃重启不打断另一台。再加一个 record_all 协调节点并行转发请求。

时间同步（两边 header.stamp 都是"采集时刻"，且在同一个主机系统时钟上）：
  - RealSense：librealsense global_time（默认开启，metadata 里 clock_domain=global_time），
    持续拟合相机硬件时钟与主机时钟的线性关系，校正漂移，不含 USB 传输延迟。
  - OAK：DepthAI 把设备时钟同步到主机 steady_clock，驱动再加上 steady -> 系统时钟的固定偏移
    （dai_time.h，直接采两个时钟，不含第一帧的管线延迟）。
  残余误差：曝光时刻定义不同（OAK 取曝光结束，RealSense 近似曝光中点，差 <= 曝光时间/2，
  默认曝光上限 3ms 即 <= 1.5ms）；OAK 的 steady->系统时钟偏移只在启动时采一次，
  NTP 调速时会慢慢偏（ppm 级，1 小时 < 几 ms），长时间录制可重启驱动重新对齐。
  RealSense 按 30Hz 全录，OAK 按 oak_record_hz 抽帧（落在绝对时间格上），离线给每组 OAK 取
  时间戳最近的 RealSense 帧，最大间隔 16.7ms（30Hz 的半帧）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LAUNCH_ARGS = [
    DeclareLaunchArgument("bag_root", default_value="~/rosbags"),
    DeclareLaunchArgument("max_duration_s", default_value="600.0"),
    DeclareLaunchArgument("oak_record_hz", default_value="4.0",
                          description="OAK 图像写盘频率（相机仍 30Hz 采集）；0 = 全录"),
    DeclareLaunchArgument("foxglove", default_value="false",
                          description="两个容器各开一个 foxglove_bridge（OAK 8765，RealSense 8766）"),
]


def _include(package, launch_file, **launch_configurations):
    """scoped + 不转发：两份 launch 有同名参数（fps、foxglove_port、bag_prefix...），必须隔离。

    参数用 GroupAction 的 launch_configurations 传：它在清空作用域之前求值；
    IncludeLaunchDescription 的 launch_arguments 在清空之后才求值，引用外层参数会报不存在。
    """
    return GroupAction(
        scoped=True,
        forwarding=False,
        launch_configurations=launch_configurations,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory(package), "launch", launch_file)),
        )],
    )


def generate_launch_description():
    common = {
        "bag_root": LaunchConfiguration("bag_root"),
        "max_duration_s": LaunchConfiguration("max_duration_s"),
        "foxglove": LaunchConfiguration("foxglove"),
    }
    oak = _include(
        "oak_ros2", "oak.launch.py",
        **common,
        record_hz=LaunchConfiguration("oak_record_hz"),
        foxglove_port="8765",
    )
    realsense = _include(
        "realsense_ros2", "recorder.launch.py",
        **common,
        foxglove_port="8766",
    )
    record_all = Node(
        package="realsense_ros2",
        executable="record_all.py",
        name="record_all",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )
    return LaunchDescription([*LAUNCH_ARGS, oak, realsense, record_all])
