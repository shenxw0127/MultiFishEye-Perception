"""Intel RealSense D435i 录制：左/右红外 + 彩色，以及三者的 camera_info。

单容器三组件（intra-process 零拷贝，录制路径不经过 DDS）：
  1. realsense2_camera::RealSenseNodeFactory —— 只开 color / infra1 / infra2 三路流，
     深度与点云关闭：录原始流用不到深度计算，省 USB 带宽和相机 ASIC 负载。
  2. rosbag2_transport::Recorder —— 只录下面 6 个话题（enable_imu:=true 时追加 /<cam>/imu）：
       /d435i/color/image_raw          RGB8  640x480@30（卷帘）
       /d435i/color/camera_info
       /d435i/infra1/image_rect_raw    mono8 640x480@30（全局快门，左）
       /d435i/infra1/camera_info
       /d435i/infra2/image_rect_raw    mono8 640x480@30（全局快门，右）
       /d435i/infra2/camera_info
  3. foxglove_bridge —— 录制过程中实时预览（默认开启，foxglove:=false 关闭）。

磁盘量参考：RGB8 922KB + 两路 mono8 各 307KB，30Hz 合计约 46MB/s（约 165GB/h）。
空间紧张可降 fps，或改录 infra 不录 color。

QoS 说明：容器内零拷贝要求 publisher 是 VOLATILE durability，所以图像 / info 话题
统一用 DEFAULT（reliable / keep_last / volatile），不能用 d435i.launch.py 里默认的
SYSTEM_DEFAULT（durability 非 VOLATILE，节点直接起不来）。零拷贝路径不经 DDS，
也就不需要 FastDDS 大数据 profile；容器外的订阅者（rviz 等）仍走 DDS 正常收图，
若外部要稳定收大图，参考 d435i.launch.py 对 fastdds_large_data.xml 的说明。

用法：
  ros2 launch realsense_ros2 recorder.launch.py
  ros2 launch realsense_ros2 recorder.launch.py bag_uri:=/data/mybag fps:=15
  ros2 launch realsense_ros2 recorder.launch.py enable_imu:=true width:=848 height:=480
停止：Ctrl+C（SIGINT），Recorder 会 finalize bag；容器已配 30s 收尾超时防止文件被截断。
"""

import os
from datetime import datetime

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode, ParameterValue

CONTAINER_NAME = "realsense_record_container"

LAUNCH_ARGS = [
    # --- 设备选择（与 d435i.launch.py 一致） ---
    DeclareLaunchArgument("camera_name", default_value="d435i",
                          description="节点名，同时决定话题前缀与 frame_id 前缀"),
    DeclareLaunchArgument("camera_namespace", default_value="",
                          description="节点命名空间，留空则话题为 /<camera_name>/..."),
    DeclareLaunchArgument("device_type", default_value="d435i",
                          description="按型号匹配设备（大小写不敏感的正则）；留空则接受任意 RealSense"),
    DeclareLaunchArgument("serial_no", default_value="",
                          description="按序列号选设备，多台相机时使用"),
    DeclareLaunchArgument("initial_reset", default_value="false",
                          description="启动前硬复位相机，用于从异常状态恢复"),
    DeclareLaunchArgument("tf_prefix", default_value="",
                          description="所有 frame_id 的额外前缀"),
    # --- 流参数：三路流同一份 profile，帧率一致好对齐 ---
    DeclareLaunchArgument("width", default_value="640"),
    DeclareLaunchArgument("height", default_value="480"),
    DeclareLaunchArgument("fps", default_value="30",
                          description="降低可减少磁盘占用（见文件头注释的数据量估算）"),
    DeclareLaunchArgument("enable_imu", default_value="false",
                          description="额外发布并录制 D435i 内置 IMU（/<camera_name>/imu）"),
    # --- 录制 ---
    DeclareLaunchArgument("bag_uri", default_value="",
                          description="bag 输出目录，留空则 ~/rosbags/<camera_name>_<时间戳>；"
                                      "已存在时自动追加 _N"),
    DeclareLaunchArgument("storage_id", default_value="mcap",
                          description="存储插件：mcap | sqlite3"),
    # --- 可视化 ---
    DeclareLaunchArgument("foxglove", default_value="true",
                          description="同容器内启动 foxglove_bridge 实时预览"),
    DeclareLaunchArgument("foxglove_port", default_value="8765"),
    DeclareLaunchArgument("log_level", default_value="info"),
]


def _str(context, name):
    return LaunchConfiguration(name).perform(context)


def _bool(context, name):
    return _str(context, name).strip().lower() in ("1", "true", "yes", "on")


def _int(context, name):
    return int(_str(context, name))


def unique_bag_uri(base):
    """若目标目录已存在，则追加 _N（N 从 1 开始）直到不冲突。"""
    if not os.path.exists(base):
        return base
    n = 1
    while os.path.exists("{}_{}".format(base, n)):
        n += 1
    return "{}_{}".format(base, n)


def launch_setup(context, *args, **kwargs):
    camera_name = _str(context, "camera_name")
    camera_namespace = _str(context, "camera_namespace")
    enable_imu = _bool(context, "enable_imu")
    profile = "{}x{}x{}".format(_int(context, "width"), _int(context, "height"), _int(context, "fps"))

    if camera_namespace:
        topic_prefix = "/{}/{}".format(camera_namespace.strip("/"), camera_name)
    else:
        topic_prefix = "/" + camera_name

    bag_uri = _str(context, "bag_uri") or os.path.join(
        os.path.expanduser("~"), "rosbags",
        camera_name + "_" + datetime.now().strftime("%Y%m%d_%H%M%S"),
    )
    bag_uri = unique_bag_uri(bag_uri)
    print("[record] 录制目录: {}".format(bag_uri))

    # 这里所有值都在 Python 侧转成目标类型，避免 launch 的 YAML 猜类型把字符串
    # 塞进 bool/int 参数导致节点启动报错（d435i.launch.py 同款做法）。
    camera_params = {
        # 设备选择
        "camera_name": camera_name,
        "camera_namespace": camera_namespace,
        "device_type": _str(context, "device_type"),
        "serial_no": _str(context, "serial_no"),
        "initial_reset": _bool(context, "initial_reset"),

        # RGB：卷帘快门，D435i 硬件限制
        "enable_color": True,
        "rgb_camera.color_profile": profile,
        "rgb_camera.color_format": "RGB8",

        # 左右红外：全局快门 mono8；深度关闭（录制用不到，见文件头注释）
        "enable_depth": False,
        "depth_module.depth_profile": profile,
        "depth_module.infra_profile": profile,
        "enable_infra1": True,
        "enable_infra2": True,
        "depth_module.infra1_format": "Y8",
        "depth_module.infra2_format": "Y8",

        # 只同步时间戳，不做深度对齐、不出点云
        "enable_sync": True,
        "align_depth.enable": False,
        "pointcloud.enable": False,

        # IMU：默认关闭；开启时 unite 成单一 /<camera_name>/imu 话题一并录制
        "enable_gyro": enable_imu,
        "enable_accel": enable_imu,
        "unite_imu_method": 2 if enable_imu else 0,  # 2 = linear_interpolation

        # QoS：intra-process 要求 VOLATILE，故用 DEFAULT（见文件头注释）
        "color_qos": "DEFAULT",
        "infra1_qos": "DEFAULT",
        "infra2_qos": "DEFAULT",
        "color_info_qos": "DEFAULT",
        "infra1_info_qos": "DEFAULT",
        "infra2_info_qos": "DEFAULT",

        # TF / 诊断
        "publish_tf": True,
        "tf_publish_rate": 0.0,  # 相机内部外参是静态的，只发 /tf_static
        "base_frame_id": "link",
        "tf_prefix": _str(context, "tf_prefix"),
        "diagnostics_period": 0.0,
    }

    realsense_node = ComposableNode(
        package="realsense2_camera",
        plugin="realsense2_camera::RealSenseNodeFactory",
        name=camera_name,
        # 必须显式给出 "/"：RealSenseNodeFactory 构造函数写死了默认命名空间
        # "camera"，不传 namespace 话题会变成 /camera/<camera_name>/...。
        namespace=camera_namespace or "/",
        parameters=[camera_params],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    record_topics = [
        topic_prefix + "/color/image_raw",
        topic_prefix + "/color/camera_info",
        topic_prefix + "/infra1/image_rect_raw",
        topic_prefix + "/infra1/camera_info",
        topic_prefix + "/infra2/image_rect_raw",
        topic_prefix + "/infra2/camera_info",
    ]
    if enable_imu:
        record_topics.append(topic_prefix + "/imu")

    recorder_node = ComposableNode(
        package="rosbag2_transport",
        plugin="rosbag2_transport::Recorder",
        name="rosbag2_recorder",
        parameters=[{
            "record.topics": record_topics,
            "record.all_topics": False,
            "record.all_services": False,
            "record.all_actions": False,
            # launch 下没有控制终端，键盘暂停/恢复用不了，关掉避免占 stdin
            "record.disable_keyboard_controls": True,
            "storage.uri": bag_uri,
            "storage.storage_id": LaunchConfiguration("storage_id"),
            # 及时落盘，Ctrl+C 收尾更快、更不易在关停超时内丢数据
            "storage.max_cache_size": 10485760,  # 10 MiB
        }],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    foxglove_node = ComposableNode(
        package="foxglove_bridge",
        plugin="foxglove_bridge::FoxgloveBridge",
        name="foxglove_bridge",
        condition=IfCondition(LaunchConfiguration("foxglove")),
        parameters=[{
            "port": ParameterValue(LaunchConfiguration("foxglove_port"), value_type=int),
            # 只服务本相机的话题，屏蔽网络上其它节点的话题，避免为未安装的消息包刷 schema 报错
            "topic_whitelist": [topic_prefix + "/.*"],
            "service_whitelist": [""],
            "param_whitelist": [""],
        }],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    container = ComposableNodeContainer(
        name=CONTAINER_NAME,
        namespace="",
        package="rclcpp_components",
        # component_container_mt 已废弃，统一入口 + executor 参数。
        # 单线程 + isolated 与 oak 的 record_fisheye4.launch.py 同款配方：重活都在
        # 各组件自己的线程里（librealsense 回调、rosbag2 写盘线程、foxglove ws），
        # executor 只做话题投递，Ctrl+C 收尾路径已被 oak 录制验证过。
        executable="component_container",
        arguments=["--executor-type", "single-threaded", "--isolated",
                   "--ros-args", "--log-level", _str(context, "log_level")],
        # 给 Recorder 足够时间在 Ctrl+C 后 finalize bag，避免被过早 SIGKILL
        sigterm_timeout="30",
        sigkill_timeout="30",
        composable_node_descriptions=[realsense_node, recorder_node, foxglove_node],
        output="screen",
        emulate_tty=True,
    )

    return [container]


def generate_launch_description():
    return LaunchDescription([*LAUNCH_ARGS, OpaqueFunction(function=launch_setup)])
