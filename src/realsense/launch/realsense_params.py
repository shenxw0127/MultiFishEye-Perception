"""RealSense D435i 录制相关的全部可调项集中在这里：录制话题、相机参数、录制组件参数。

改录制内容 / 相机设置只动这个文件。命令行覆盖照旧：

    ros2 launch realsense_ros2 recorder.launch.py fps:=15 exposure_us:=5000 bag_root:=/data/bags
"""

import glob
import os

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# ---------------------------------------------------------------------------
# 录制话题：(话题, 消息类型, QoS, 启用条件)
#
#   话题     不以 "/" 开头的按相机前缀 /<camera_namespace>/<camera_name>/ 展开，
#            以 "/" 开头的原样使用（如 /tf_static）。
#   QoS      reliable    —— 普通数据流，与驱动的 DEFAULT QoS 匹配（见 camera_parameters 的 *_qos）
#            best_effort —— 能匹配任意发布端（发布端 QoS 不确定时用）
#            latched     —— 只发一次的静态数据（/tf_static、extrinsics），transient_local
#   启用条件 None 总是录；"imu" 仅 enable_imu 时录；"align_depth" 仅 align_depth 时录。
#
# 录制结束时组件会打印每个话题收到的条数，0 条的会标出来，方便核对这里写得对不对。
# ---------------------------------------------------------------------------
RECORD_TOPICS = [
    # RGB（卷帘快门，RGB8）
    ("color/image_raw", "sensor_msgs/msg/Image", "reliable", None),
    ("color/camera_info", "sensor_msgs/msg/CameraInfo", "reliable", None),
    # 深度（Z16 -> 16UC1，单位 mm；坐标系 = infra1，即左红外）
    ("depth/image_rect_raw", "sensor_msgs/msg/Image", "reliable", None),
    ("depth/camera_info", "sensor_msgs/msg/CameraInfo", "reliable", None),
    # 外参：深度 -> RGB 的旋转 + 平移（工厂标定值，column-major 旋转，单位米）
    ("extrinsics/depth_to_color", "realsense2_camera_msgs/msg/Extrinsics", "latched", None),
    # IMU（accel 100Hz + gyro 200Hz 线性插值合成单一话题，约 200Hz）；enable_imu:=false 时自动不录
    ("imu", "sensor_msgs/msg/Imu", "reliable", "imu"),
    # IMU 内参（噪声方差、零偏）与 深度 -> accel / gyro 外参（D435i 出厂无 IMU 标定，是型号默认值）
    ("accel/imu_info", "realsense2_camera_msgs/msg/IMUInfo", "latched", "imu"),
    ("gyro/imu_info", "realsense2_camera_msgs/msg/IMUInfo", "latched", "imu"),
    ("extrinsics/depth_to_accel", "realsense2_camera_msgs/msg/Extrinsics", "latched", "imu"),
    ("extrinsics/depth_to_gyro", "realsense2_camera_msgs/msg/Extrinsics", "latched", "imu"),

    # 动捕刚体位姿不在这里录：由独立的 vrpn_recorder.launch.py 录成单独的 bag（相机拔插 / 崩溃不影响动捕）。
    # 完整 TF 树（各 frame 间的静态外参），latched，体积可忽略
    ("/tf_static", "tf2_msgs/msg/TFMessage", "latched", None),

    # ---- 以下当前不录，需要时去掉 "#"。红外还要 enable_infra:=true ----
    # 左 / 右红外（全局快门，mono8）
    # ("infra1/image_rect_raw", "sensor_msgs/msg/Image", "reliable", None),
    # ("infra1/camera_info", "sensor_msgs/msg/CameraInfo", "reliable", None),
    # ("infra2/image_rect_raw", "sensor_msgs/msg/Image", "reliable", None),
    # ("infra2/camera_info", "sensor_msgs/msg/CameraInfo", "reliable", None),
    # ("extrinsics/depth_to_infra1", "realsense2_camera_msgs/msg/Extrinsics", "latched", None),
    # ("extrinsics/depth_to_infra2", "realsense2_camera_msgs/msg/Extrinsics", "latched", None),
    # 完整 TF 树：<cam>_link -> color / depth / infra1 / infra2 / imu 各 frame 及 optical frame
    # 对齐到 RGB 的深度（逐像素与 color 对应，内参同 color）；align_depth:=true 时才发布
    ("aligned_depth_to_color/image_raw", "sensor_msgs/msg/Image", "reliable", "align_depth"),
    ("aligned_depth_to_color/camera_info", "sensor_msgs/msg/CameraInfo", "reliable", "align_depth"),
]

# 相机 launch 参数（默认值都是字符串，camera_parameters 里转成目标类型）
CAMERA_DEFAULTS = {
    # 设备选择（与 d435i.launch.py 一致）
    "camera_name": "d435i",
    "camera_namespace": "",
    "device_type": "d435i",
    "serial_no": "",
    "initial_reset": "false",
    "tf_prefix": "",
    # 流参数：所有流同一份 profile，帧率一致好对齐
    "width": "848",
    "height": "480",
    "fps": "30",
    "enable_imu": "true",
    "enable_infra": "false",
    "align_depth": "false",
    # 曝光（见 exposure_params）
    "exposure_us": "6000",
    "rgb_gain": "128",
    # 热插拔：驱动在相机不在时一直等，拔掉后每隔 reconnect_timeout 秒重新找设备
    "reconnect_timeout": "2.0",
}

CAMERA_DESCRIPTIONS = {
    "camera_name": "节点名，同时决定话题前缀与 frame_id 前缀",
    "camera_namespace": "节点命名空间，留空则话题为 /<camera_name>/...",
    "device_type": "按型号匹配设备（大小写不敏感的正则）；留空则接受任意 RealSense",
    "serial_no": "按序列号选设备，多台相机时使用",
    "initial_reset": "启动前硬复位相机，用于从异常状态恢复",
    "tf_prefix": "所有 frame_id 的额外前缀",
    "fps": "降低可减少磁盘占用（见 recorder.launch.py 文件头的数据量估算）",
    "enable_imu": "发布并录制 D435i 内置 IMU（/<camera_name>/imu 及其内参 / 外参），false 关闭",
    "enable_infra": "发布左右红外图像；关闭时深度照常输出（深度在相机芯片里算），省 USB 带宽",
    "align_depth": "额外发布并录制对齐到 RGB 的深度（aligned_depth_to_color），占额外 CPU 与磁盘",
    "exposure_us": "曝光上限（µs）。红外保持自动曝光但上限锁在此值，暗了由自动增益补；"
                   "RGB 改为手动曝光取此值；<=0 恢复全自动",
    "rgb_gain": "RGB 手动曝光时的增益（0~128），画面偏暗就调大",
    "reconnect_timeout": "相机拔掉后重新搜索设备的间隔（秒）",
}

# 录制组件（TopicRecorderComponent）参数。
# 触发：ros2 service call /<camera_name>/record oak_interfaces/srv/Record "{duration_s: 30}"
RECORDER_DEFAULTS = {
    "bag_root": "~/rosbags",
    "bag_prefix": "d435i",
    "storage_id": "mcap",
    "max_duration_s": "600.0",
}


def _str(context, name):
    return LaunchConfiguration(name).perform(context)


def _bool(context, name):
    return _str(context, name).strip().lower() in ("1", "true", "yes", "on")


def _int(context, name):
    return int(_str(context, name))


def _float(context, name):
    return float(_str(context, name))


def realsense_launch_args():
    """相机 + 录制组件的 DeclareLaunchArgument 列表。"""
    return [
        DeclareLaunchArgument(name, default_value=value, description=CAMERA_DESCRIPTIONS.get(name, ""))
        for name, value in {**CAMERA_DEFAULTS, **RECORDER_DEFAULTS}.items()
    ]


def topic_prefix(context):
    camera_name = _str(context, "camera_name")
    camera_namespace = _str(context, "camera_namespace").strip("/")
    return "/{}/{}".format(camera_namespace, camera_name) if camera_namespace else "/" + camera_name


def realsense_imu_blocked():
    """返回 RealSense IMU 中当前用户不可写的 IIO 使能节点列表（空列表 = 可用或没接相机）。

    D435i 的 accel/gyro 是 USB HID -> IIO 设备，librealsense 要写
    scan_elements/*_en。没装 udev 规则时这些节点是 root 0644，Motion Module
    打开失败后 realsense 节点会反复硬复位相机，连图像都录不到，所以启动前先查。
    """
    blocked = []
    for dev in glob.glob("/sys/bus/iio/devices/iio:device*"):
        try:
            with open(os.path.join(dev, "name")) as f:
                name = f.read().strip()
        except OSError:
            continue
        # 只看挂在 Intel(8086) USB 设备下的，排除笔记本自带的 HID 传感器
        if name not in ("accel_3d", "gyro_3d") or ":8086:" not in os.path.realpath(dev):
            continue
        for en in glob.glob(os.path.join(dev, "scan_elements", "*_en")):
            if not os.access(en, os.W_OK):
                blocked.append(en)
    return blocked


def imu_enabled(context):
    """enable_imu 且 IIO 节点可写。不可写时打印警告并关闭 IMU，保证图像照常录制。"""
    if not _bool(context, "enable_imu"):
        return False
    blocked = realsense_imu_blocked()
    if blocked:
        print("[record] 警告: IMU 的 IIO 节点无写权限（如 {}），已自动关闭 IMU，只录图像。\n"
              "[record] 修复: 安装 librealsense 的 99-realsense-libusb.rules 到 "
              "/etc/udev/rules.d/ 后重新插拔相机（见 recorder.launch.py 文件头）".format(blocked[0]))
        return False
    return True


def exposure_params(exposure_us, rgb_gain):
    """把"曝光上限 µs"翻成两个模块各自的参数。exposure_us <= 0 表示不动，全交给自动曝光。

    两个模块单位不同（rs-enumerate-devices -o 实测）：
      - Stereo Module（红外，全局快门）：Exposure 单位 µs，1~165000，默认自动曝光。
        保留自动曝光，只开 Auto Exposure Limit 把上限锁住，亮度不够时自动增益来补，
        既去运动模糊又不会在暗处直接黑掉。
      - RGB Camera（卷帘快门）：Exposure 单位 100µs，1~10000，没有曝光上限选项，
        只能关自动曝光改手动，增益由 rgb_gain 给定。
    """
    if exposure_us <= 0:
        return {}
    return {
        "depth_module.enable_auto_exposure": True,
        "depth_module.auto_exposure_limit_toggle": True,
        "depth_module.auto_exposure_limit": exposure_us,
        "rgb_camera.enable_auto_exposure": False,
        "rgb_camera.exposure": max(1, round(exposure_us / 100)),
        "rgb_camera.gain": rgb_gain,
    }


def camera_parameters(context, enable_imu):
    """RealSenseNodeFactory 的参数字典。

    所有值都在 Python 侧转成目标类型，避免 launch 的 YAML 猜类型把字符串
    塞进 bool/int 参数导致节点启动报错（d435i.launch.py 同款做法）。
    """
    profile = "{}x{}x{}".format(_int(context, "width"), _int(context, "height"), _int(context, "fps"))
    align_depth = _bool(context, "align_depth")
    return {
        # 设备选择
        "camera_name": _str(context, "camera_name"),
        "camera_namespace": _str(context, "camera_namespace"),
        "device_type": _str(context, "device_type"),
        "serial_no": _str(context, "serial_no"),
        "initial_reset": _bool(context, "initial_reset"),

        # 热插拔：-1 = 启动时相机不在就一直等；拔掉后驱动自己 reset 并按间隔重新找设备，
        # 插回后用同一套参数重新开流，话题名不变，录制组件的订阅会自动重新匹配。
        "wait_for_device_timeout": -1.0,
        "reconnect_timeout": _float(context, "reconnect_timeout"),

        # RGB：卷帘快门，D435i 硬件限制
        "enable_color": True,
        "rgb_camera.color_profile": profile,
        "rgb_camera.color_format": "RGB8",

        # 深度 + 左右红外（全局快门 mono8，enable_infra 控制）。深度同时是驱动发布 extrinsics 的
        # 基准流，关掉深度就拿不到 depth_to_color 外参。
        "enable_depth": True,
        "enable_infra1": _bool(context, "enable_infra"),
        "enable_infra2": _bool(context, "enable_infra"),
        "depth_module.depth_profile": profile,
        "depth_module.infra_profile": profile,
        "depth_module.depth_format": "Z16",
        "depth_module.infra1_format": "Y8",
        "depth_module.infra2_format": "Y8",

        # 同一帧集内时间戳同步；不出点云
        "enable_sync": True,
        "align_depth.enable": align_depth,
        "pointcloud.enable": False,

        # 曝光（见 exposure_params）
        **exposure_params(_int(context, "exposure_us"), _int(context, "rgb_gain")),

        # IMU：unite 成单一 /<camera_name>/imu 话题
        "enable_gyro": enable_imu,
        "enable_accel": enable_imu,
        "unite_imu_method": 2 if enable_imu else 0,  # 2 = linear_interpolation

        # QoS：DEFAULT = reliable / keep_last / volatile，与 RECORD_TOPICS 里的 reliable 匹配；
        # 也满足容器内 intra-process 要求的 VOLATILE（SYSTEM_DEFAULT 会让节点起不来）。
        "color_qos": "DEFAULT",
        "depth_qos": "DEFAULT",
        "infra1_qos": "DEFAULT",
        "infra2_qos": "DEFAULT",
        "color_info_qos": "DEFAULT",
        "depth_info_qos": "DEFAULT",
        "infra1_info_qos": "DEFAULT",
        "infra2_info_qos": "DEFAULT",

        # TF / 诊断
        "publish_tf": True,
        "tf_publish_rate": 0.0,  # 相机内部外参是静态的，只发 /tf_static
        "base_frame_id": "link",
        "tf_prefix": _str(context, "tf_prefix"),
        "diagnostics_period": 0.0,
    }


def record_topics(context, enable_imu):
    """按启用条件筛选 RECORD_TOPICS 并展开相机前缀，返回 [(topic, type, qos), ...]。"""
    enabled = {None, *(["imu"] if enable_imu else []), *(["align_depth"] if _bool(context, "align_depth") else [])}
    prefix = topic_prefix(context)
    return [
        (topic if topic.startswith("/") else "{}/{}".format(prefix, topic), msg_type, qos)
        for topic, msg_type, qos, when in RECORD_TOPICS
        if when in enabled
    ]


def recorder_parameters(context, enable_imu):
    """TopicRecorderComponent 的参数字典。"""
    topics = record_topics(context, enable_imu)
    return {
        "bag_root": _str(context, "bag_root"),
        "bag_prefix": _str(context, "bag_prefix"),
        "storage_id": _str(context, "storage_id"),
        "max_duration_s": _float(context, "max_duration_s"),
        "service_name": topic_prefix(context) + "/record",
        "record_topics": [t[0] for t in topics],
        "record_types": [t[1] for t in topics],
        "record_qos": [t[2] for t in topics],
    }
