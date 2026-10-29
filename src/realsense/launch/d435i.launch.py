"""Intel RealSense D435i：30Hz 彩色点云 + RGB 图像（外加全局快门左红外图）。

关于「全局快门」的硬件事实：D435i 只有深度用的左右红外 imager 是全局快门，
RGB 模块是卷帘快门，这由硬件决定、SDK 无法切换。所以：
  - 彩色点云的几何部分天然是全局快门（来自双目红外）；
  - RGB 图像只能是卷帘快门；
  - 需要全局快门灰度图的下游请订阅 infra1（默认已开启，mono8）。
真正 RGB 也是全局快门的型号是 D455。

相机节点复用官方 realsense2_camera::RealSenseNodeFactory 组件，装在 composable
container 里，方便把本项目其它组件加进同一进程做 intra-process 零拷贝。

发布的主要话题（camera_name 默认 d435i）：
  /d435i/color/image_raw              RGB8  848x480 @30Hz（卷帘）
  /d435i/color/camera_info
  /d435i/depth/color/points           PointCloud2 XYZRGB @30Hz  <- 彩色点云
  /d435i/infra1/image_rect_raw        mono8 848x480 @30Hz（全局快门）
  /d435i/depth/image_rect_raw         Z16 深度图
  /d435i/aligned_depth_to_color/image_raw  对齐到彩色的深度图

坐标系：base = d435i_link，点云与深度在 d435i_depth_optical_frame，
彩色在 d435i_color_optical_frame，红外在 d435i_infra1_optical_frame。

用法：
  ros2 launch realsense_ros2 d435i.launch.py
  ros2 launch realsense_ros2 d435i.launch.py foxglove:=true
  ros2 launch realsense_ros2 d435i.launch.py width:=640 height:=480 enable_imu:=true

跑满 30Hz 的前提（DDS 传输，别删）：
本 launch 会给相机进程设 FASTRTPS_DEFAULT_PROFILES_FILE 指向
config/fastdds_large_data.xml。Fast DDS 默认共享内存段只有 512KB，而单帧
color rgb8 是 1.2MB、depth Z16 是 814KB、点云最大 13MB（848x480），超出后退回 UDP 分片，
又被内核 net.core.rmem_max(默认 208KB) 卡住大量丢片。实测默认配置下
color 5.3Hz / depth 5.2Hz / 点云 4.2Hz，加载该 profile 后三者都是 29.98Hz
（infra1 只有 407KB，没超，两种配置都是 30Hz）。以上实测是 640x480，详见该 xml 的头部注释。

注意 QoS：点云默认 SENSOR_DATA（best-effort）。`ros2 topic hz` 内部用的就是
sensor_data QoS，可以直接测；但 `ros2 topic echo` 和 rviz2 默认 reliable，
需要显式改成 best-effort（`ros2 topic echo --qos-reliability best_effort`；
rviz2 把 Reliability Policy 设为 Best Effort）。注意 `ros2 topic hz` 没有
这组开关，它固定用 sensor_data QoS。图像默认 SYSTEM_DEFAULT（reliable），
直接 echo / rviz 即可。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

CONTAINER_NAME = "realsense_container"

# Fast DDS 大数据 profile：不加载它，color / depth / 点云会从 30Hz 掉到 5Hz 左右。
# 原因与实测数据见该文件头部注释。
DEFAULT_DDS_PROFILE = os.path.join(
    get_package_share_directory("realsense_ros2"), "config", "fastdds_large_data.xml"
)

# librealsense 的 RS2_STREAM_COLOR，用作点云的贴图流（即用 RGB 给点上色）。
RS2_STREAM_COLOR = 2

LAUNCH_ARGS = [
    # --- 设备选择 ---
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
    # --- 流参数：30Hz 是硬要求，分辨率可调 ---
    DeclareLaunchArgument("width", default_value="848"),  # Intel 给 D435 推荐的深度分辨率
    DeclareLaunchArgument("height", default_value="480"),
    DeclareLaunchArgument("fps", default_value="30"),
    # --- 可选流 ---
    DeclareLaunchArgument("enable_infra1", default_value="true",
                          description="发布左红外图（全局快门 mono8）"),
    DeclareLaunchArgument("enable_infra2", default_value="false",
                          description="发布右红外图（全局快门 mono8）"),
    DeclareLaunchArgument("enable_imu", default_value="false",
                          description="发布 D435i 内置 IMU（gyro/accel 及插值后的 ~/imu）"),
    # --- 点云 ---
    DeclareLaunchArgument("align_depth", default_value="true",
                          description="深度对齐到彩色；点云基于对齐后的深度图，色彩逐像素对应"),
    DeclareLaunchArgument("ordered_pointcloud", default_value="false",
                          description="true 则输出有序点云 (height=H, width=W)，含无效点"),
    # --- QoS / 诊断 / 可视化 ---
    DeclareLaunchArgument("image_qos", default_value="SYSTEM_DEFAULT",
                          description="图像话题 QoS：SYSTEM_DEFAULT | DEFAULT | SENSOR_DATA | ..."),
    DeclareLaunchArgument("info_qos", default_value="SYSTEM_DEFAULT",
                          description="camera_info 与 metadata 话题 QoS"),
    DeclareLaunchArgument("pointcloud_qos", default_value="SENSOR_DATA",
                          description="点云话题 QoS；848x480 XYZRGB @30Hz 约 195MB/s，建议 best-effort"),
    DeclareLaunchArgument("diagnostics_period", default_value="0.0",
                          description=">0 则以该周期(秒)发布 /diagnostics，可看到各流实际帧率"),
    DeclareLaunchArgument("intra_process_comms", default_value="false",
                          description="容器内零拷贝；只有把消费者组件也放进本容器才有意义，"
                                      "开启时会自动把 QoS 换成 VOLATILE（见 _volatile）"),
    DeclareLaunchArgument("dds_profile", default_value=DEFAULT_DDS_PROFILE,
                          description="Fast DDS profile 路径，跑满 30Hz 必需；置空则不设置。"
                                      "若外部已 export FASTRTPS_DEFAULT_PROFILES_FILE，则以外部为准"),
    DeclareLaunchArgument("log_level", default_value="info"),
    DeclareLaunchArgument("foxglove", default_value="false",
                          description="同容器内启动 foxglove_bridge 以便实时查看"),
    DeclareLaunchArgument("foxglove_port", default_value="8765"),
]


def _str(context, name):
    return LaunchConfiguration(name).perform(context)


def _bool(context, name):
    return _str(context, name).strip().lower() in ("1", "true", "yes", "on")


def _int(context, name):
    return int(_str(context, name))


def _float(context, name):
    return float(_str(context, name))


def _volatile(qos):
    """把 SYSTEM_DEFAULT 换成 DEFAULT，供 intra-process 使用。

    rclcpp 要求启用 intra-process 的 publisher 必须是 VOLATILE durability，而
    SYSTEM_DEFAULT 的 durability 不是 VOLATILE，wrapper 建 publisher 时会直接抛
    "intraprocess communication allowed only with volatile durability"，相机根本起不来。
    DEFAULT 同样是 reliable / keep_last，只是 durability 明确为 VOLATILE。
    （wrapper 自己已经给 extrinsics、imu_info 这些 latched 话题单独关掉了
    intra-process，所以只需要处理这几个可配的 QoS。）
    """
    return "DEFAULT" if qos.strip().upper() == "SYSTEM_DEFAULT" else qos


def launch_setup(context, *args, **kwargs):
    camera_name = _str(context, "camera_name")
    camera_namespace = _str(context, "camera_namespace")
    # 三个流用同一份 <宽>x<高>x<帧率>，保证 color / depth / infra 帧率一致好同步。
    profile = "{}x{}x{}".format(_int(context, "width"), _int(context, "height"), _int(context, "fps"))
    enable_imu = _bool(context, "enable_imu")
    intra_process = _bool(context, "intra_process_comms")

    image_qos = _str(context, "image_qos")
    info_qos = _str(context, "info_qos")
    pointcloud_qos = _str(context, "pointcloud_qos")
    if intra_process:
        image_qos, info_qos, pointcloud_qos = map(_volatile, (image_qos, info_qos, pointcloud_qos))

    # 这里所有值都在 Python 侧转成了目标类型，不依赖 launch 的 YAML 猜类型，
    # 避免把字符串塞进 bool/int 参数导致节点启动报错。
    params = {
        # 设备选择
        "camera_name": camera_name,
        "camera_namespace": camera_namespace,
        "device_type": _str(context, "device_type"),
        "serial_no": _str(context, "serial_no"),
        "initial_reset": _bool(context, "initial_reset"),

        # RGB：卷帘快门，这是 D435i 的硬件限制
        "enable_color": True,
        "rgb_camera.color_profile": profile,
        "rgb_camera.color_format": "RGB8",

        # 深度：来自左右红外双目，全局快门
        "enable_depth": True,
        "depth_module.depth_profile": profile,
        "depth_module.depth_format": "Z16",

        # 红外原图：全局快门灰度，给需要 GS 图像的下游用
        "enable_infra1": _bool(context, "enable_infra1"),
        "enable_infra2": _bool(context, "enable_infra2"),
        "depth_module.infra_profile": profile,
        "depth_module.infra1_format": "Y8",
        "depth_module.infra2_format": "Y8",

        # 彩色点云
        "enable_sync": True,  # 让 librealsense 把 color/depth/infra 凑成同一时间戳的 frameset
        "align_depth.enable": _bool(context, "align_depth"),
        "pointcloud.enable": True,
        "pointcloud.stream_filter": RS2_STREAM_COLOR,
        "pointcloud.stream_index_filter": 0,
        "pointcloud.ordered_pc": _bool(context, "ordered_pointcloud"),
        # false：只保留能取到颜色的点（深度 FOV 87x58 比彩色 69x42 大，边缘会被裁掉）
        "pointcloud.allow_no_texture_points": False,
        "pointcloud.pointcloud_qos": pointcloud_qos,

        # IMU：D435i 自带，默认关闭；本 launch 的职责是图像与点云
        "enable_gyro": enable_imu,
        "enable_accel": enable_imu,
        "unite_imu_method": 2 if enable_imu else 0,  # 2 = linear_interpolation

        # QoS / TF / 诊断
        # *_info_qos 同时管 camera_info 和 metadata 话题，必须一起设；漏掉它就会在
        # 开启 intra-process 时因为 SYSTEM_DEFAULT durability 而启动失败。
        "color_qos": image_qos,
        "depth_qos": image_qos,
        "infra1_qos": image_qos,
        "infra2_qos": image_qos,
        "color_info_qos": info_qos,
        "depth_info_qos": info_qos,
        "infra1_info_qos": info_qos,
        "infra2_info_qos": info_qos,
        "publish_tf": True,
        "tf_publish_rate": 0.0,  # 相机内部外参是静态的，只发 /tf_static
        "base_frame_id": "link",
        "tf_prefix": _str(context, "tf_prefix"),
        "diagnostics_period": _float(context, "diagnostics_period"),
    }

    realsense_node = ComposableNode(
        package="realsense2_camera",
        plugin="realsense2_camera::RealSenseNodeFactory",
        name=camera_name,
        # 必须显式给出 "/"：RealSenseNodeFactory 的构造函数写死了默认命名空间
        # "camera"，不传 namespace 的话话题会变成 /camera/<camera_name>/...。
        namespace=camera_namespace or "/",
        parameters=[params],
        extra_arguments=[{"use_intra_process_comms": intra_process}],
    )

    container = ComposableNodeContainer(
        name=CONTAINER_NAME,
        namespace="",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[realsense_node],
        arguments=["--ros-args", "--log-level", _str(context, "log_level")],
        output="screen",
        emulate_tty=True,
    )

    topic_prefix = "/{}/{}".format(camera_namespace.strip("/"), camera_name) if camera_namespace else "/" + camera_name
    foxglove_node = ComposableNode(
        package="foxglove_bridge",
        plugin="foxglove_bridge::FoxgloveBridge",
        name="foxglove_bridge",
        parameters=[{
            "port": _int(context, "foxglove_port"),
            # 只服务本相机的话题，屏蔽网络上其它节点的话题，避免为未安装的消息包刷 schema 报错
            "topic_whitelist": [topic_prefix + "/.*"],
            "service_whitelist": [""],
            "param_whitelist": [""],
        }],
        extra_arguments=[{"use_intra_process_comms": intra_process}],
    )

    load_foxglove = LoadComposableNodes(
        target_container="/" + CONTAINER_NAME,
        condition=IfCondition(LaunchConfiguration("foxglove")),
        composable_node_descriptions=[foxglove_node],
    )

    actions = []
    # 必须在容器启动前设好：Fast DDS 只在参与者创建时读一次 profile 文件。
    # 外部已经 export 过就不覆盖，尊重用户/上层 launch 的全局 DDS 配置。
    dds_profile = _str(context, "dds_profile")
    if dds_profile and not os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"):
        actions.append(SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", dds_profile))

    actions += [container, load_foxglove]
    return actions


def generate_launch_description():
    return LaunchDescription([*LAUNCH_ARGS, OpaqueFunction(function=launch_setup)])
