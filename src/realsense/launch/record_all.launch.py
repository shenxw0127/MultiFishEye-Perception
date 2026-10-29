"""OAK 四目鱼眼 + RealSense D435i + VRPN 动捕联合录制（无硬件同步，靠时间戳软同步）。

    ros2 launch realsense_ros2 record_all.launch.py
    ros2 launch realsense_ros2 record_all.launch.py oak_record_hz:=4.0 bag_root:=/data/bags
    ros2 launch realsense_ros2 record_all.launch.py vrpn_server:=192.168.3.14                  # 顺带起 vrpn_mocap
    ros2 launch realsense_ros2 record_all.launch.py rigid_bodies:=d435i_body,oak_body          # 动捕里的刚体名不同时
    ros2 launch realsense_ros2 record_all.launch.py foxglove:=true oak_launch:=oak_usb2_mono.launch.py

录制（三路同时开录，bag 放进同一个 session 目录）：
    ros2 service call /record_all oak_interfaces/srv/Record "{duration_s: 30}"
    -> <bag_root>/session_<时间戳>/{oak,d435i,vrpn}
       vrpn bag 里每个刚体一个话题：默认 /vrpn_mocap/realsense/pose 与 /vrpn_mocap/eyes4/pose（两台相机各自的位姿）
单独录某一路照旧：/oak/record（oak_ns 改了则为 /<oak_ns>/record）、/d435i/record、/vrpn/record。vrpn:=false 则只录两台相机。

进程结构：三路各一个容器（oak.launch.py、recorder.launch.py、vrpn_recorder.launch.py 原样 include），
互不影响：一台相机拔插 / 崩溃重启不打断另一路，动捕数据也不依赖相机。再加一个 record_all 协调节点并行转发请求。

时间同步（两边 header.stamp 都是"采集时刻"，且在同一个主机系统时钟上）：
  - RealSense：librealsense global_time（默认开启，metadata 里 clock_domain=global_time），
    持续拟合相机硬件时钟与主机时钟的线性关系，校正漂移，不含 USB 传输延迟。
  - OAK：DepthAI 把设备时钟同步到主机 steady_clock，驱动再加上 steady -> 系统时钟的固定偏移
    （dai_time.h，直接采两个时钟，不含第一帧的管线延迟）。
  残余误差：曝光时刻定义不同（OAK 取曝光结束，RealSense 近似曝光中点，差 <= 曝光时间/2，
  默认曝光上限 3ms 即 <= 1.5ms）；OAK 的 steady->系统时钟偏移只在启动时采一次，
  NTP 调速时会慢慢偏（ppm 级，1 小时 < 几 ms），长时间录制可重启驱动重新对齐。
  - VRPN：vrpn_mocap 默认 use_vrpn_timestamps=false，stamp = 本机收到数据的系统时钟，同一时钟但比采集时刻
    晚 动捕处理 + 网络 的延迟（通常几 ms ~ 十几 ms），需离线标定这个固定偏移（scripts/check_session_sync.py）。
  三个 bag 的 log time 都是消息 header.stamp（采集 / 收到时刻，不是录制端写盘时刻），可以直接按 bag 时间合并、同放。
  RealSense 按 30Hz 全录；OAK 默认也全录，设 oak_record_hz:=4 时抽帧（落在绝对时间格上）。离线给每帧 OAK 取
  时间戳最近的 RealSense 帧，最大间隔 16.7ms（30Hz 的半帧）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LAUNCH_ARGS = [
    DeclareLaunchArgument("bag_root", default_value="~/rosbags"),
    DeclareLaunchArgument("max_duration_s", default_value="600.0"),
    DeclareLaunchArgument("oak_record_hz", default_value="999.0",
                          description="OAK 图像写盘频率，默认 999 = 满帧；标定时设 4，相机仍 30Hz 采集、只写 4Hz"),
    DeclareLaunchArgument("oak_launch", default_value="oak.launch.py",
                          description="oak_ros2 包里用哪个 launch 起 OAK：oak.launch.py（USB3 彩色 400p）"
                                      "或 oak_usb2_mono.launch.py（USB2 + 厂商黑白 + 设备端 MJPEG）"),
    DeclareLaunchArgument("oak_ns", default_value="oak",
                          description="OAK 话题命名空间：/<oak_ns>/CAM_A/image，录制服务 /<oak_ns>/record"),
    DeclareLaunchArgument("foxglove", default_value="false",
                          description="两个相机容器各开一个 foxglove_bridge（OAK 8765，RealSense 8766）。"
                                      "OAK 那个不设白名单，连 8765 就能同时看到两台相机和 /isp_meta"),
    DeclareLaunchArgument("vrpn", default_value="true",
                          description="同时录 VRPN 动捕刚体位姿（独立 bag）；false 只录两台相机"),
    DeclareLaunchArgument("rigid_bodies", default_value="realsense,eyes4",
                          description="要录的动捕刚体名（动捕软件里的名字），逗号分隔，话题 /vrpn_mocap/<名>/pose；"
                                      "默认两台相机各一个刚体"),
    DeclareLaunchArgument("vrpn_server", default_value="",
                          description="非空则同时启动 vrpn_mocap 客户端；留空 = 假设已在别处运行（如 ~/start_vrpn.sh）"),
    DeclareLaunchArgument("use_vrpn_timestamps", default_value="false"),
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


def launch_setup(context, *args, **kwargs):
    common = {
        "bag_root": LaunchConfiguration("bag_root"),
        "max_duration_s": LaunchConfiguration("max_duration_s"),
    }
    camera_common = {**common, "foxglove": LaunchConfiguration("foxglove")}
    oak = _include(
        "oak_ros2", LaunchConfiguration("oak_launch").perform(context),
        **camera_common,
        record_hz=LaunchConfiguration("oak_record_hz"),
        ns=LaunchConfiguration("oak_ns"),
        foxglove_port="8765",
    )
    realsense = _include(
        "realsense_ros2", "recorder.launch.py",
        **camera_common,
        foxglove_port="8766",
    )
    actions = [oak, realsense]
    # record_all 转发的子服务：{bag 子目录名: 服务名}，子目录名同时是 session 目录下的 bag 名
    oak_ns = LaunchConfiguration("oak_ns").perform(context).strip("/")
    targets = [f"oak:/{oak_ns}/record", "d435i:/d435i/record"]

    if LaunchConfiguration("vrpn").perform(context).strip().lower() in ("1", "true", "yes", "on"):
        actions.append(_include(
            "realsense_ros2", "vrpn_recorder.launch.py",
            **common,
            rigid_bodies=LaunchConfiguration("rigid_bodies"),
            vrpn_server=LaunchConfiguration("vrpn_server"),
            use_vrpn_timestamps=LaunchConfiguration("use_vrpn_timestamps"),
        ))
        targets.append("vrpn:/vrpn/record")

    actions.append(Node(
        package="realsense_ros2",
        executable="record_all.py",
        name="record_all",
        output="screen",
        parameters=[{"targets": targets}],
        respawn=True,
        respawn_delay=2.0,
    ))
    return actions


def generate_launch_description():
    return LaunchDescription([*LAUNCH_ARGS, OpaqueFunction(function=launch_setup)])
