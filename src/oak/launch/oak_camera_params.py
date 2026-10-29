"""OAK 相机/IMU 驱动组件（OakFfcCameraImuComponent）和录制组件的全部参数集中在这里。

改相机设置只动这个文件，所有 launch 都会跟着变。命令行覆盖照旧：

    ros2 launch oak_ros2 oak.launch.py resolution:=800p auto_exposure_limit_us:=5000

launch 文件的用法：

    FIXED = {"compressed": False, "lazy_publisher": False}   # 本 launch 写死、不暴露为参数
    launch_args = [*oak_launch_args(FIXED, resolution="400p"), ...]  # 关键字 = 改默认值
    launch_args = [ns_launch_arg(), *oak_launch_args(FIXED), ...]  # ns 必须先于 tf_prefix 声明
    ComposableNode(..., namespace=LaunchConfiguration("ns"), parameters=oak_parameters(FIXED))

话题命名空间 ns（默认 oak）：驱动和录制组件挂在 /<ns> 下，话题为 /oak/CAM_A/image、/oak/imu、
/oak/isp_meta，录制服务为 /oak/record。tf_prefix 默认跟随 ns，多实例时 frame_id 也不会撞。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

# 驱动基础参数：设备、分辨率、帧率、同步、队列、IMU。
# resolution 默认 800p，与 config/camera/*-camchain.yaml 的标定分辨率 [1280, 800] 一致；
# 400p（800p sensor + ISP 1/2 缩放）视场不变但内参减半，依赖标定的 launch 不要直接切。
NS_DEFAULT = "oak"

DRIVER_DEFAULTS = {
    # 默认取 ns 的值（DeclareLaunchArgument 执行时才求值，所以 ns 要先声明）
    "tf_prefix": LaunchConfiguration("ns"),
    "camera_name": "ov9782",
    "camera_type": "color",
    "resolution": "400p",
    "fps": "30",
    "oak_fw_uri": "",
    # 绑定的 OAK MxId，空 = 第一台空闲设备；多台同时接入时必须填
    "device_id": "",
    "imu_config_path": os.path.join(get_package_share_directory("oak_ros2"), "config", "imu.yaml"),
    "imu_correction_enabled": "true",
    "compressed": "false",
    "usb_speed": "super",
    "xlink_mjpeg": "false",
    "jpeg_quality": "95",
    "imu_hz": "200",
    # ALL = 启动时自动检测插着的相机；也可写显式列表，如 "[CAM_A, CAM_B]"（没插的会被跳过并告警）。
    # "cam_board_sockets": "ALL",
    "cam_board_sockets": "[CAM_A, CAM_B, CAM_C, CAM_D]",

    "sync_threshold_ms": "30",
    "image_queue_size": "1",
    "imu_queue_size": "50",
    "lazy_publisher": "true",
    "reconnect_interval_ms": "1000",
    "fps_display": "true",
}

# ISP 参数（曝光、增益、白平衡等）。
# 当前配置：固定曝光上限 + 自动增益 + 自动白平衡。
# manual_exposure_us / manual_iso 留 0 让固件跑自动曝光，再用 auto_exposure_limit_us
# 把曝光时间压住，光线不足时 AE 顶到上限后只能靠拉 ISO；manual_wb_kelvin 留 0 走 AWB。
#
# 想回到之前的全手动曝光，改成：
#   manual_exposure_us 5000、manual_iso 400、auto_exposure_limit_us 0、manual_wb_kelvin 3500
CAMERA_DEFAULTS = {
    "manual_exposure_us": "0",
    "manual_iso": "0",
    "exposure_compensation": "0",
    "auto_exposure_limit_us": "3000",
    "manual_wb_kelvin": "0",
    "brightness": "0",
    "contrast": "0",
    "saturation": "0",
    "sharpness": "1",
    "luma_denoise": "1",
    "chroma_denoise": "1",
    "manual_focus": "-1",
}

DESCRIPTIONS = {
    "tf_prefix": "frame_id 前缀（<tf_prefix>_CAM_A_optical_frame），默认同 ns",
    "device_id": "OAK MxId；空=连第一台空闲设备。查询：lsusb -v -d 03e7: | grep iSerial",
    "resolution": "400p/480p/720p/800p/1080p/1200p/4k；color 下 400p 为 800p + ISP 1/2 缩放",
    "imu_correction_enabled": "apply accel bias/scale + gyro bias correction from imu_config_path "
    "(set false to publish raw IMU)",
    "compressed": "true 走 video 流 MJPEG 压缩；false 走 preview 输出 bgr8 原图",
    "usb_speed": "super=允许 USB3；high=强制协商 USB2（避免 USB3 射频干扰 GNSS）",
    "xlink_mjpeg": "设备端 MJPEG 过 XLink，主机解码回 mono8/bgr8；USB2 下 4 路 400p@30 必须开",
    "jpeg_quality": "设备端 MJPEG 质量 1..100；95 时 4×400p mono 约 7.5 MB/s",
    "lazy_publisher": "无订阅者时不发布；录制/下游组件消费时应关闭",
    "reconnect_interval_ms": "相机不在或断连后的重连间隔",
    "fps_display": "在启动终端单行刷新各路相机实际帧率（低于目标 95% 标红）；没有终端时自动关闭",
}

ALL_DEFAULTS = {**DRIVER_DEFAULTS, **CAMERA_DEFAULTS}

# 录制组件（BagRecorderComponent）参数。cam_board_sockets 与驱动共用同一个 launch 参数。
# 触发：ros2 service call /<ns>/record oak_interfaces/srv/Record "{duration_s: 30}"（默认 /oak/record）
RECORDER_DEFAULTS = {
    "bag_root": "~/rosbags",
    "bag_prefix": "fisheye4",
    "storage_id": "mcap",
    "jpeg_quality": "90",
    "jpeg_threads": "2",
    "max_queue": "60",
    "max_duration_s": "600.0",
    # 图像写盘频率。默认 999 = 满帧（任何 >= 相机 fps 的值都是全录）；标定时 record_hz:=4，
    # 相机仍按 fps 采集，录制只按绝对时间格每 1/record_hz 秒留一组。
    "record_hz": "999.0",
}


def ns_launch_arg(default=NS_DEFAULT):
    """话题命名空间 launch 参数，放在 oak_launch_args 之前。"""
    return DeclareLaunchArgument(
        "ns", default_value=default, description="话题命名空间：/<ns>/CAM_A/image、/<ns>/imu、/<ns>/record")


def oak_launch_args(fixed=None, **overrides):
    """驱动组件的 DeclareLaunchArgument 列表。

    fixed: 本 launch 写死的参数（不声明成 launch 参数，避免命令行传了却不生效）。
    overrides: 改默认值，如 resolution="400p"。
    """
    fixed = fixed or {}
    unknown = set(overrides) - set(ALL_DEFAULTS)
    if unknown:
        raise KeyError(f"unknown OAK launch args: {sorted(unknown)}")
    defaults = {**ALL_DEFAULTS, **overrides}
    return [
        DeclareLaunchArgument(name, default_value=value, description=DESCRIPTIONS.get(name, ""))
        for name, value in defaults.items()
        if name not in fixed
    ]


def oak_parameters(fixed=None):
    """传给 OakFfcCameraImuComponent 的 parameters= 字典列表，fixed 中的参数用写死的值。"""
    fixed = fixed or {}
    unknown = set(fixed) - set(ALL_DEFAULTS)
    if unknown:
        raise KeyError(f"unknown OAK parameters: {sorted(unknown)}")
    return [
        {name: fixed[name] if name in fixed else LaunchConfiguration(name)}
        for name in ALL_DEFAULTS
    ]


def recorder_launch_args():
    """录制组件的 DeclareLaunchArgument 列表。"""
    return [DeclareLaunchArgument(name, default_value=value) for name, value in RECORDER_DEFAULTS.items()]


def recorder_parameters(**overrides):
    """传给 BagRecorderComponent 的 parameters= 字典列表。overrides 替换对应参数的值（如多机时按设备给插座）。"""
    params = {
        **{name: LaunchConfiguration(name) for name in RECORDER_DEFAULTS},
        "cam_board_sockets": LaunchConfiguration("cam_board_sockets"),
        # 相机帧率与驱动共用 fps 参数，用来判断 record_hz 是否需要抽帧
        "camera_fps": LaunchConfiguration("fps"),
    }
    unknown = set(overrides) - set(params)
    if unknown:
        raise KeyError(f"unknown recorder parameters: {sorted(unknown)}")
    return [{name: value} for name, value in {**params, **overrides}.items()]
