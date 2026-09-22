"""相机 ISP 参数（曝光、增益、白平衡等）集中在这里。

改相机设置只动这个文件，所有 launch 都会跟着变。命令行覆盖照旧：

    ros2 launch oak_ros2 oak.launch.py auto_exposure_limit_us:=5000
"""

from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

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


def camera_launch_args():
    """相机参数对应的 DeclareLaunchArgument 列表。"""
    return [
        DeclareLaunchArgument(name, default_value=value)
        for name, value in CAMERA_DEFAULTS.items()
    ]


def camera_parameters():
    """传给 ComposableNode 的 parameters= 字典列表。"""
    return [{name: LaunchConfiguration(name)} for name in CAMERA_DEFAULTS]
