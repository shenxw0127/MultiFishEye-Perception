"""动捕（VRPN）刚体位姿录制：独立进程 + 独立 bag，与 OAK / RealSense 用同一套触发方式。

    ros2 launch realsense_ros2 vrpn_recorder.launch.py                              # vrpn_mocap 已在别处运行
    ros2 launch realsense_ros2 vrpn_recorder.launch.py vrpn_server:=192.168.3.14    # 顺带启动 vrpn_mocap 客户端
    ros2 launch realsense_ros2 vrpn_recorder.launch.py rigid_bodies:=realsense,eyes4  # 刚体名按动捕软件里的改

录制：
    ros2 service call /vrpn/record oak_interfaces/srv/Record "{duration_s: 30}"
    -> <bag_root>/vrpn_<时间戳>，每个刚体一个话题 /vrpn_mocap/<刚体名>/pose（geometry_msgs/PoseStamped）
默认录两个刚体：realsense（D435i 上的 marker 刚体）和 eyes4（OAK 鱼眼模组上的），同一个 bag。

为什么单独一个进程 / 一个 bag：动捕不依赖任何相机硬件，相机拔插或驱动崩溃不应该带走动捕数据；
联合录制时由 record_all 把三个 bag 放进同一个 session 目录（record_all.launch.py）。

时间戳（与另外两个 bag 对齐用）：bag 的 log time = 消息 header.stamp（TopicRecorderComponent 统一做法）。
vrpn_mocap 默认 use_vrpn_timestamps=false，stamp = 本机收到数据时的系统时钟，和相机的"采集时刻"同一时钟，
但多了 动捕系统处理 + 网络 的延迟（通常几 ms ~ 十几 ms，需要离线标定，见 check_session_sync.py）。
把 use_vrpn_timestamps:=true 则用 VRPN 服务器的时间，仅当动捕服务器与本机时钟同步（同一 NTP / PTP）时才有意义。

刚体名 = 动捕软件里的 rigid body 名；vrpn_mocap 把名字里的非字母数字字符换成 '_' 再当话题名
（如 "OAK-1" -> /vrpn_mocap/OAK_1/pose），这里做同样的转换，按动捕软件里的原名填即可。
vrpn_mocap 是收到刚体数据后才创建话题，录制端用 generic subscription，订阅可以先于发布者存在，
所以刚体晚出现也能录到；开录 2s 后某个刚体仍无数据，录制端会告警并列出当前存在的 PoseStamped 话题。
"""

import os
import re

from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode

CONTAINER_NAME = "vrpn_record_container"
# 容器刚起来时 launch 侧 load_node 客户端与容器的 DDS 匹配可能还没完成（原因见 recorder.launch.py），延迟加载。
LOAD_DELAY_SEC = 2.0

# 两台相机各贴一个刚体；改这里时 record_all.launch.py 的同名默认值一起改
DEFAULT_RIGID_BODIES = "realsense,eyes4"

LAUNCH_ARGS = [
    DeclareLaunchArgument("bag_root", default_value="~/rosbags"),
    DeclareLaunchArgument("bag_prefix", default_value="vrpn"),
    DeclareLaunchArgument("storage_id", default_value="mcap"),
    DeclareLaunchArgument("max_duration_s", default_value="600.0"),
    DeclareLaunchArgument("rigid_bodies", default_value=DEFAULT_RIGID_BODIES,
                          description="要录的刚体名（动捕软件里的名字），逗号分隔；话题为 /vrpn_mocap/<名>/pose"),
    DeclareLaunchArgument("vrpn_server", default_value="",
                          description="非空则同时启动 vrpn_mocap 客户端连接该服务器；留空 = 假设已在别处运行"),
    DeclareLaunchArgument("vrpn_port", default_value="3883"),
    DeclareLaunchArgument("use_vrpn_timestamps", default_value="false",
                          description="true 用 VRPN 服务器时间戳（要求动捕服务器与本机时钟同步）；false 用本机收到时间"),
    DeclareLaunchArgument("respawn_delay", default_value="3.0"),
    DeclareLaunchArgument("log_level", default_value="info"),
]


def _str(context, name):
    return LaunchConfiguration(name).perform(context)


def vrpn_topic_name(rigid_body):
    """与 vrpn_mocap Tracker::ValidNodeName 一致：非字母数字字符替换为 '_'。"""
    return re.sub(r"[^0-9A-Za-z]", "_", rigid_body)


def rigid_body_names(context):
    names = [n.strip() for n in _str(context, "rigid_bodies").split(",") if n.strip()]
    if not names:
        raise RuntimeError("rigid_bodies must contain at least one rigid body name")
    topic_names = [vrpn_topic_name(n) for n in names]
    if len(set(topic_names)) != len(topic_names):
        raise RuntimeError("rigid_bodies map to duplicate topic names: {}".format(topic_names))
    return topic_names


def vrpn_client(context):
    """vrpn_mocap 客户端节点。沿用 ~/start_vrpn.sh 的做法：它的 .so 在默认库路径之外。"""
    prefix = get_package_prefix("vrpn_mocap")
    lib_dir = os.path.join(prefix, "lib", "vrpn_mocap")
    ld_path = lib_dir + os.pathsep + os.environ.get("LD_LIBRARY_PATH", "")
    return Node(
        package="vrpn_mocap",
        executable="client_node",
        namespace="vrpn_mocap",
        name="vrpn_mocap_client_node",
        parameters=[
            os.path.join(prefix, "share", "vrpn_mocap", "config", "client.yaml"),
            {
                "server": _str(context, "vrpn_server"),
                "port": int(_str(context, "vrpn_port")),
                "use_vrpn_timestamps": _str(context, "use_vrpn_timestamps").strip().lower() in ("1", "true", "yes", "on"),
            },
        ],
        additional_env={"LD_LIBRARY_PATH": ld_path},
        respawn=True,
        respawn_delay=float(_str(context, "respawn_delay")),
        output="screen",
    )


def launch_setup(context, *args, **kwargs):
    topics = ["/vrpn_mocap/{}/pose".format(name) for name in rigid_body_names(context)]
    print("[vrpn] 触发: ros2 service call /vrpn/record oak_interfaces/srv/Record \"{duration_s: 30}\"")
    print("[vrpn] 录制话题:\n  " + "\n  ".join(topics))

    recorder_node = ComposableNode(
        package="realsense_ros2",
        plugin="realsense_ros2::TopicRecorderComponent",
        name="vrpn_recorder",
        parameters=[{
            "bag_root": _str(context, "bag_root"),
            "bag_prefix": _str(context, "bag_prefix"),
            "storage_id": _str(context, "storage_id"),
            "max_duration_s": float(_str(context, "max_duration_s")),
            "service_name": "/vrpn/record",
            "record_topics": topics,
            "record_types": ["geometry_msgs/msg/PoseStamped"] * len(topics),
            # vrpn_mocap 的发布端是 sensor-data QoS（best_effort），只有 best_effort 订阅能匹配
            "record_qos": ["best_effort"] * len(topics),
        }],
    )

    container = ComposableNodeContainer(
        name=CONTAINER_NAME,
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        arguments=["--executor-type", "single-threaded", "--isolated",
                   "--ros-args", "--log-level", _str(context, "log_level")],
        # 给录制组件足够时间在 Ctrl+C 后关闭 bag
        sigterm_timeout="30",
        sigkill_timeout="30",
        respawn=True,
        respawn_delay=float(_str(context, "respawn_delay")),
        output="screen",
        emulate_tty=True,
    )

    # 每次容器进程启动（含 respawn）都延迟加载一遍组件；action 不能复用，每次新建。
    def load_components(event, launch_context):
        return [TimerAction(
            period=LOAD_DELAY_SEC,
            actions=[LoadComposableNodes(target_container=CONTAINER_NAME,
                                         composable_node_descriptions=[recorder_node])],
        )]

    actions = [RegisterEventHandler(OnProcessStart(target_action=container, on_start=load_components)), container]
    if _str(context, "vrpn_server"):
        actions.append(vrpn_client(context))
    return actions


def generate_launch_description():
    return LaunchDescription([*LAUNCH_ARGS, OpaqueFunction(function=launch_setup)])
