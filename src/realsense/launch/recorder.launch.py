"""Intel RealSense D435i：热插拔驱动 + 按需定时录制 + 可选 foxglove（与 oak.launch.py 同一套用法）。

    ros2 launch realsense_ros2 recorder.launch.py
    ros2 launch realsense_ros2 recorder.launch.py fps:=15 bag_root:=/data/bags foxglove:=false

录制（相机常驻运行，收到请求才开始录，到时自动收尾）：
    ros2 service call /d435i/record oak_interfaces/srv/Record "{duration_s: 30}"

录哪些话题、相机参数都在同目录的 realsense_params.py 里改（RECORD_TOPICS）。默认录：
    color/image_raw + camera_info             RGB8  640x480@30（卷帘）
    depth/image_rect_raw + camera_info        16UC1 640x480@30（mm，坐标系 = 左红外）
    infra1|infra2/image_rect_raw + camera_info mono8 640x480@30（全局快门，左 / 右）
    imu                                       accel + gyro 插值合成（enable_imu:=false 时不录）
    extrinsics/depth_to_{color,infra1,infra2} 工厂标定外参（旋转 + 平移），latched
    /tf_static                                完整相机 TF 树，latched

外参说明：depth 与 infra1 同一坐标系，所以 depth_to_color 就是左红外 -> RGB，
depth_to_infra2 就是左 -> 右红外（平移约 50mm 基线）。驱动以深度流为基准算外参，
因此深度流必须开着，这也是旧版（关深度、不录 /tf_static）bag 里缺外参的原因。

单容器三组件：
  1. realsense2_camera::RealSenseNodeFactory —— wait_for_device_timeout=-1：启动时没插相机就一直等；
     运行中拔掉后驱动自己 reset，按 reconnect_timeout 间隔重新找设备，插回后用同一套参数重新开流。
  2. realsense_ros2::TopicRecorderComponent —— /<camera_name>/record 触发，只在录制期间订阅。
     相机在录制中被拔掉再插回，订阅会自动重新匹配，bag 继续写（中间缺一段）。
  3. foxglove_bridge —— 实时预览（foxglove:=false 关闭）。

自启动：容器进程异常退出（librealsense 在拔插时偶发崩溃）后 respawn_delay 秒自动重启并重新加载
三个组件；Ctrl+C 正常退出不会触发。崩溃时正在录的 bag 来不及写 metadata，可用
`ros2 bag reindex <bag>` 修复。

磁盘量参考：RGB8 922KB + Z16 614KB + 两路 mono8 各 307KB，30Hz 合计约 64MB/s（约 230GB/h）。

IMU 前置条件：D435i 的 IMU 走 HID/IIO（/sys/bus/iio/devices/iio:device*），默认只有 root
可写，必须装 librealsense 的 udev 规则（config/99-realsense-libusb.rules，与 librealsense
版本对应）到 /etc/udev/rules.d/ 并重新插拔相机。否则 Motion Module 打开时报
"in_anglvel_x_en ... Permission denied"，节点会反复硬复位相机，连图像也录不到。
launch 启动前会检查权限，不可写时自动关闭 IMU 并打印警告（启动时相机没插则检查不到）。

停止：Ctrl+C（SIGINT），录制中会先收尾 bag；容器已配 30s 收尾超时防止文件被截断。
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterValue

# 相机与录制参数、录制话题集中在同目录的 realsense_params.py
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from realsense_params import (  # noqa: E402
    camera_parameters,
    imu_enabled,
    realsense_launch_args,
    recorder_parameters,
    topic_prefix,
)

CONTAINER_NAME = "realsense_record_container"
# 容器起来后延迟多久再加载组件。容器刚启动时 Fast DDS 还没把 launch 侧 load_node
# 客户端的 response reader 匹配上，rmw 只等 100ms 就丢弃响应
# （"failed to send response ... (timeout)"），而 launch_ros 按顺序加载、
# 等不到响应就永远卡住，后面的组件根本不会被加载。
LOAD_DELAY_SEC = 2.0

LAUNCH_ARGS = [
    *realsense_launch_args(),
    DeclareLaunchArgument("respawn_delay", default_value="3.0",
                          description="容器异常退出后多少秒重启"),
    DeclareLaunchArgument("foxglove", default_value="true",
                          description="同容器内启动 foxglove_bridge 实时预览"),
    DeclareLaunchArgument("foxglove_port", default_value="8765"),
    DeclareLaunchArgument("log_level", default_value="info"),
]


def launch_setup(context, *args, **kwargs):
    camera_name = LaunchConfiguration("camera_name").perform(context)
    camera_namespace = LaunchConfiguration("camera_namespace").perform(context)
    enable_imu = imu_enabled(context)
    prefix = topic_prefix(context)

    realsense_node = ComposableNode(
        package="realsense2_camera",
        plugin="realsense2_camera::RealSenseNodeFactory",
        name=camera_name,
        # 必须显式给出 "/"：RealSenseNodeFactory 构造函数写死了默认命名空间
        # "camera"，不传 namespace 话题会变成 /camera/<camera_name>/...。
        namespace=camera_namespace or "/",
        parameters=[camera_parameters(context, enable_imu)],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    recorder_params = recorder_parameters(context, enable_imu)
    print("[record] 触发: ros2 service call {} oak_interfaces/srv/Record \"{{duration_s: 30}}\"".format(
        recorder_params["service_name"]))
    print("[record] 录制话题:\n  " + "\n  ".join(recorder_params["record_topics"]))
    recorder_node = ComposableNode(
        package="realsense_ros2",
        plugin="realsense_ros2::TopicRecorderComponent",
        name="{}_recorder".format(camera_name),
        parameters=[recorder_params],
    )

    foxglove_node = ComposableNode(
        package="foxglove_bridge",
        plugin="foxglove_bridge::FoxgloveBridge",
        name="foxglove_bridge",
        condition=IfCondition(LaunchConfiguration("foxglove")),
        parameters=[{
            "port": ParameterValue(LaunchConfiguration("foxglove_port"), value_type=int),
            # 只服务本相机的话题，屏蔽网络上其它节点的话题，避免为未安装的消息包刷 schema 报错
            "topic_whitelist": [prefix + "/.*"],
            "service_whitelist": [""],
            "param_whitelist": [""],
        }],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    container = ComposableNodeContainer(
        name=CONTAINER_NAME,
        namespace="",
        package="rclcpp_components",
        # 单线程 + isolated 与 oak.launch.py 同款：每个组件一个独立 executor 线程，
        # 重活在各自线程里（librealsense 回调、rosbag2 落盘线程、foxglove ws）。
        executable="component_container",
        arguments=["--executor-type", "single-threaded", "--isolated",
                   "--ros-args", "--log-level", LaunchConfiguration("log_level").perform(context)],
        # 给录制组件足够时间在 Ctrl+C 后关闭 bag，避免被过早 SIGKILL
        sigterm_timeout="30",
        sigkill_timeout="30",
        respawn=True,
        respawn_delay=float(LaunchConfiguration("respawn_delay").perform(context)),
        output="screen",
        emulate_tty=True,
    )

    # 每次容器进程启动（含 respawn）都延迟加载一遍组件（延迟原因见 LOAD_DELAY_SEC）。
    # 每次都要新建 action 实例，TimerAction / LoadComposableNodes 不能重复执行同一个。
    # Recorder 放第一个：即便某个响应仍然丢失，卡住的也只是后面的组件，录制服务已经在了。
    def load_components(event, launch_context):
        return [TimerAction(
            period=LOAD_DELAY_SEC,
            actions=[LoadComposableNodes(
                target_container=CONTAINER_NAME,
                composable_node_descriptions=[recorder_node, foxglove_node, realsense_node],
            )],
        )]

    return [
        RegisterEventHandler(OnProcessStart(target_action=container, on_start=load_components)),
        container,
    ]


def generate_launch_description():
    return LaunchDescription([*LAUNCH_ARGS, OpaqueFunction(function=launch_setup)])
