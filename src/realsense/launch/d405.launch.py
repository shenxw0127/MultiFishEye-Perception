"""Intel RealSense D405：用 Intel 官方算法输出深度图（+ 可选 RGB / 点云）。

"官方算法"分两段，都不是本项目自己写的：
  1. 立体匹配：D405 的深度由相机内 D4 ASIC 硬件算好，USB 上传下来的就是 Z16 深度；
     匹配参数由 visual_preset / json_file_path（Intel Advanced Mode 预设）控制。
  2. 后处理：librealsense 自带的 post-processing filter，由 realsense2_camera 驱动
     在发布前按 Intel 白皮书《Depth Post-Processing for Intel RealSense D400》推荐的顺序执行：
         decimation -> depth->disparity -> spatial -> temporal -> disparity->depth -> hole_filling
     默认开 disparity + spatial + temporal；decimation 会降分辨率、hole_filling 会凭空补值，
     默认关闭，需要时用参数打开。
所以 /d405/depth/image_rect_raw 发布的就是经过官方滤波后的深度图。

D405 与 D435i 的硬件差异（决定了下面参数怎么写）：
  - 没有红外投射器、没有 IMU；靠被动双目，适合 7cm ~ 50cm 的近距离。
  - 只有一个 Stereo Module：RGB 来自左 imager 自带的 ISP，所以彩色参数是
    depth_module.color_profile / color_format，而不是 D435i 的 rgb_camera.*。
  - RGB 与深度出自同一颗全局快门左 imager。实测 extrinsics/depth_to_color 的旋转是
    单位阵（偏差 <1e-3）、平移约 1e-5 m，所以默认不做 align_depth，深度和 RGB 逐像素已基本对应。

发布的主要话题（camera_name 默认 d405）：
  /d405/depth/image_rect_raw          16UC1 深度图（单位 mm，见下），已过官方后处理
  /d405/depth/camera_info
  /d405/color/image_raw               RGB8（enable_color:=true，默认开）
  /d405/color/camera_info
  /d405/extrinsics/depth_to_color
  /d405/depth/color/points            点云（cloud:=true 时）：有 RGB 时为 XYZRGB，enable_color:=false 时为 XYZ
                                      （XYZ 模式启动时驱动会打一条 "No matching stream for texture" 警告，属正常）
  /d405/aligned_depth_to_color/image_raw  （align_depth:=true 时）

深度单位：毫米（16UC1，ROS REP 118 惯例）。realsense2_camera 不把 depth_units 暴露成参数，
实测 clip_distance:=0.3 时深度最大值为 297，确认是 mm 而不是 D405 可选的 0.1mm。
D405 超过 ~0.5m 的深度噪声很大（室内实测 p95 能到 7m），只关心近处时建议设 clip_distance。

实测（FW 5.15.1.55，librealsense 2.59.1，USB 3.2，默认参数 1280x720x30 + 官方滤波链）：
  depth 29.9Hz / color 30.0Hz。
  cloud:=true：XYZRGB 29.5Hz，室内约 11.6 万有效点、每帧 2.3MB；有序点云固定 1280x720、每帧 18.4MB，28.4Hz。

用法：
  ros2 launch realsense_ros2 d405.launch.py
  ros2 launch realsense_ros2 d405.launch.py foxglove:=true
  ros2 launch realsense_ros2 d405.launch.py visual_preset:=3            # High Accuracy 预设
  ros2 launch realsense_ros2 d405.launch.py temporal_filter:=false      # 相机快速运动时关掉时域滤波，避免拖影
  ros2 launch realsense_ros2 d405.launch.py width:=848 height:=480 fps:=90
  ros2 launch realsense_ros2 d405.launch.py cloud:=true clip_distance:=0.5   # 发布点云，裁掉 0.5m 外的噪点

DDS：同 d435i.launch.py，相机进程默认加载 config/fastdds_large_data.xml。
1280x720 下单帧 depth 1.8MB、color 2.7MB，远超 Fast DDS 默认 512KB 共享内存段，不加载会掉帧。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode

CONTAINER_NAME = "d405_container"

DEFAULT_DDS_PROFILE = os.path.join(
    get_package_share_directory("realsense_ros2"), "config", "fastdds_large_data.xml"
)

# librealsense 的流类型，用作点云的贴图流：COLOR 出 XYZRGB，ANY 表示不贴图、只出 XYZ
RS2_STREAM_ANY = 0
RS2_STREAM_COLOR = 2

LAUNCH_ARGS = [
    # --- 设备选择 ---
    DeclareLaunchArgument("camera_name", default_value="d405",
                          description="节点名，同时决定话题前缀与 frame_id 前缀"),
    DeclareLaunchArgument("camera_namespace", default_value="",
                          description="节点命名空间，留空则话题为 /<camera_name>/..."),
    DeclareLaunchArgument("device_type", default_value="d405",
                          description="按型号匹配设备（大小写不敏感的正则）；同时接着 D435i 时靠它选中 D405"),
    DeclareLaunchArgument("serial_no", default_value="",
                          description="按序列号选设备，多台 D405 时使用"),
    DeclareLaunchArgument("initial_reset", default_value="false",
                          description="启动前硬复位相机，用于从异常状态恢复"),
    DeclareLaunchArgument("tf_prefix", default_value="",
                          description="所有 frame_id 的额外前缀"),
    DeclareLaunchArgument("reconnect_timeout", default_value="2.0",
                          description="相机拔掉后重新搜索设备的间隔（秒）"),
    # --- 流参数：1280x720 是 Intel 给 D405 的推荐深度分辨率（该分辨率最高 30Hz；848x480 及以下可到 90Hz）---
    DeclareLaunchArgument("width", default_value="1280"),
    DeclareLaunchArgument("height", default_value="720"),
    DeclareLaunchArgument("fps", default_value="30"),
    DeclareLaunchArgument("enable_color", default_value="true",
                          description="同时发布 RGB（与深度同一颗左 imager，同分辨率同帧率）"),
    # --- 官方立体匹配参数（ASIC 内） ---
    DeclareLaunchArgument("visual_preset", default_value="-1",
                          description="Advanced Mode 预设：1 Default / 2 Hand / 3 High Accuracy / "
                                      "4 High Density / 5 Medium Density / 7 Edge Enhancement；-1 不改动"),
    DeclareLaunchArgument("json_file_path", default_value="",
                          description="Intel RealSense Viewer 导出的 Advanced Mode json，优先级高于 visual_preset"),
    DeclareLaunchArgument("clip_distance", default_value="-1.0",
                          description=">0 则把超过该距离（米）的深度置 0；D405 有效量程约 0.07~0.5m"),
    # --- 官方后处理滤波器（librealsense） ---
    DeclareLaunchArgument("decimation_filter", default_value="false",
                          description="降采样（分辨率除以 magnitude），开启后深度与 RGB 分辨率不再一致"),
    DeclareLaunchArgument("decimation_magnitude", default_value="2"),
    DeclareLaunchArgument("disparity_filter", default_value="true",
                          description="spatial/temporal 前转视差域、之后转回深度（Intel 推荐，近处平滑更合理）"),
    DeclareLaunchArgument("spatial_filter", default_value="true",
                          description="保边空间平滑"),
    DeclareLaunchArgument("spatial_magnitude", default_value="2", description="迭代次数 1~5"),
    DeclareLaunchArgument("spatial_alpha", default_value="0.5", description="平滑系数 0.25~1，越小越平滑"),
    DeclareLaunchArgument("spatial_delta", default_value="20", description="保边阈值 1~50，越小越保边"),
    DeclareLaunchArgument("spatial_holes_fill", default_value="0",
                          description="空间补洞半径：0 关 / 1 2px / 2 4px / 3 8px / 4 16px / 5 不限"),
    DeclareLaunchArgument("temporal_filter", default_value="true",
                          description="时域平滑；相机或物体快速运动时会有拖影，可关闭"),
    DeclareLaunchArgument("temporal_alpha", default_value="0.4", description="0~1，越小历史权重越大"),
    DeclareLaunchArgument("temporal_delta", default_value="20", description="1~100，超过即视为边缘不平滑"),
    DeclareLaunchArgument("temporal_persistency", default_value="3",
                          description="持续性补洞：0 关 / 3 最近4帧有2帧有效（默认）/ 8 始终沿用"),
    DeclareLaunchArgument("hole_filling_filter", default_value="false",
                          description="按邻域补齐剩余空洞（补的是估计值，不是测量值）"),
    DeclareLaunchArgument("hole_filling_mode", default_value="1",
                          description="0 取左侧 / 1 取邻域最远 / 2 取邻域最近"),
    # --- 可选输出 ---
    DeclareLaunchArgument("align_depth", default_value="false",
                          description="额外发布对齐到 RGB 的深度；D405 两者同源，一般不需要"),
    DeclareLaunchArgument("cloud", default_value="false",
                          description="发布点云 /<camera_name>/depth/color/points（基于滤波后的深度）"),
    DeclareLaunchArgument("cloud_ordered", default_value="false",
                          description="true 则输出有序点云 (height=H, width=W)，含无效点"),
    # --- QoS / 诊断 / 可视化 ---
    DeclareLaunchArgument("image_qos", default_value="SYSTEM_DEFAULT",
                          description="图像话题 QoS：SYSTEM_DEFAULT | DEFAULT | SENSOR_DATA | ..."),
    DeclareLaunchArgument("info_qos", default_value="SYSTEM_DEFAULT",
                          description="camera_info 与 metadata 话题 QoS"),
    DeclareLaunchArgument("cloud_qos", default_value="SENSOR_DATA",
                          description="点云话题 QoS，默认 best-effort；rviz2 需把 Reliability 设为 Best Effort"),
    DeclareLaunchArgument("diagnostics_period", default_value="0.0",
                          description=">0 则以该周期(秒)发布 /diagnostics，可看到各流实际帧率"),
    DeclareLaunchArgument("intra_process_comms", default_value="false",
                          description="容器内零拷贝；开启时自动把 SYSTEM_DEFAULT QoS 换成 DEFAULT"),
    DeclareLaunchArgument("dds_profile", default_value=DEFAULT_DDS_PROFILE,
                          description="Fast DDS profile 路径；置空则不设置。外部已 export 时以外部为准"),
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
    """intra-process 要求 VOLATILE durability，SYSTEM_DEFAULT 不满足（详见 d435i.launch.py）。"""
    return "DEFAULT" if qos.strip().upper() == "SYSTEM_DEFAULT" else qos


def filter_parameters(context):
    """librealsense 后处理滤波器参数。执行顺序由驱动固定，与这里的书写顺序无关。"""
    return {
        "decimation_filter.enable": _bool(context, "decimation_filter"),
        "decimation_filter.filter_magnitude": _int(context, "decimation_magnitude"),
        # depth->disparity 与 disparity->depth 必须成对开：驱动 4.59 不会自动转回，
        # 只开前者时深度帧停在 DISPARITY32，发布端报 "Format DISPARITY32 is not supported" 并整帧丢弃。
        "disparity_filter.enable": _bool(context, "disparity_filter"),
        "disparity_to_depth.enable": _bool(context, "disparity_filter"),
        "spatial_filter.enable": _bool(context, "spatial_filter"),
        "spatial_filter.filter_magnitude": _int(context, "spatial_magnitude"),
        "spatial_filter.filter_smooth_alpha": _float(context, "spatial_alpha"),
        "spatial_filter.filter_smooth_delta": _int(context, "spatial_delta"),
        "spatial_filter.holes_fill": _int(context, "spatial_holes_fill"),
        "temporal_filter.enable": _bool(context, "temporal_filter"),
        "temporal_filter.filter_smooth_alpha": _float(context, "temporal_alpha"),
        "temporal_filter.filter_smooth_delta": _int(context, "temporal_delta"),
        "temporal_filter.holes_fill": _int(context, "temporal_persistency"),
        "hole_filling_filter.enable": _bool(context, "hole_filling_filter"),
        "hole_filling_filter.holes_fill": _int(context, "hole_filling_mode"),
    }


def launch_setup(context, *args, **kwargs):
    camera_name = _str(context, "camera_name")
    camera_namespace = _str(context, "camera_namespace")
    profile = "{}x{}x{}".format(_int(context, "width"), _int(context, "height"), _int(context, "fps"))
    enable_color = _bool(context, "enable_color")
    cloud = _bool(context, "cloud")
    align_depth = _bool(context, "align_depth")
    intra_process = _bool(context, "intra_process_comms")
    if align_depth and not enable_color:
        raise RuntimeError("align_depth 需要 RGB，请同时设 enable_color:=true")

    image_qos = _str(context, "image_qos")
    info_qos = _str(context, "info_qos")
    cloud_qos = _str(context, "cloud_qos")
    if intra_process:
        image_qos, info_qos, cloud_qos = map(_volatile, (image_qos, info_qos, cloud_qos))

    # 值都在 Python 侧转成目标类型，避免 launch 的 YAML 猜类型出错（同 d435i.launch.py）。
    params = {
        # 设备选择 / 热插拔
        "camera_name": camera_name,
        "camera_namespace": camera_namespace,
        "device_type": _str(context, "device_type"),
        "serial_no": _str(context, "serial_no"),
        "initial_reset": _bool(context, "initial_reset"),
        "wait_for_device_timeout": -1.0,
        "reconnect_timeout": _float(context, "reconnect_timeout"),

        # 深度：ASIC 立体匹配结果
        "enable_depth": True,
        "depth_module.depth_profile": profile,
        "depth_module.depth_format": "Z16",
        "json_file_path": _str(context, "json_file_path"),
        "clip_distance": _float(context, "clip_distance"),

        # RGB：D405 的彩色流属于 Stereo Module，参数挂在 depth_module 下
        "enable_color": enable_color,
        "depth_module.color_profile": profile,
        "depth_module.color_format": "RGB8",

        # D405 的红外是 RGB 格式的原图，深度输出用不到
        "enable_infra1": False,
        "enable_infra2": False,

        # 官方后处理
        **filter_parameters(context),

        # 深度与 RGB 凑成同一时间戳的 frameset
        "enable_sync": enable_color,
        "align_depth.enable": align_depth,
        "pointcloud.enable": cloud,
        "pointcloud.stream_filter": RS2_STREAM_COLOR if enable_color else RS2_STREAM_ANY,
        "pointcloud.stream_index_filter": 0 if enable_color else -1,
        "pointcloud.ordered_pc": _bool(context, "cloud_ordered"),
        "pointcloud.pointcloud_qos": cloud_qos,

        # QoS / TF / 诊断
        "color_qos": image_qos,
        "depth_qos": image_qos,
        "color_info_qos": info_qos,
        "depth_info_qos": info_qos,
        "publish_tf": True,
        "tf_publish_rate": 0.0,  # 内部外参是静态的，只发 /tf_static
        "base_frame_id": "link",
        "tf_prefix": _str(context, "tf_prefix"),
        "diagnostics_period": _float(context, "diagnostics_period"),
    }
    # -1 表示不碰预设，沿用相机当前值（参数范围是 0~7，传 -1 会被驱动拒绝）
    visual_preset = _int(context, "visual_preset")
    if visual_preset >= 0:
        params["depth_module.visual_preset"] = visual_preset

    realsense_node = ComposableNode(
        package="realsense2_camera",
        plugin="realsense2_camera::RealSenseNodeFactory",
        name=camera_name,
        # RealSenseNodeFactory 默认命名空间写死为 "camera"，必须显式给 "/"
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
    # Fast DDS 只在参与者创建时读一次 profile，必须在容器启动前设好
    dds_profile = _str(context, "dds_profile")
    if dds_profile and not os.environ.get("FASTRTPS_DEFAULT_PROFILES_FILE"):
        actions.append(SetEnvironmentVariable("FASTRTPS_DEFAULT_PROFILES_FILE", dds_profile))

    actions += [container, load_foxglove]
    return actions


def generate_launch_description():
    return LaunchDescription([*LAUNCH_ARGS, OpaqueFunction(function=launch_setup)])
