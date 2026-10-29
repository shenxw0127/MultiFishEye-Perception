"""AR0234 传感器的 oak.launch.py 预设（参考厂商 oak_ffc_sync_ar0234.launch.py）。

    ros2 launch oak_ros2 ar.launch.py
    ros2 launch oak_ros2 ar.launch.py foxglove:=false fps:=60

- camera_name=ar0234：驱动据此走脚本 FSIN 同步（LEON_CSS 上用 GPIO 打帧同步脉冲，
  所有 sensor 均为 INPUT），而不是 OV9782 的“首路 sensor 输出 FSIN”。
- camera_type=color / resolution=400p：AR0234 取 1200p 全幅，ISP 硬件缩 1/3 到 640x400，
  视场不变（见 config.cpp resolveResolution）。4×400p bgr8@30 约 92 MB/s，USB3 直接传 raw。
- 要原生 1920x1200：resolution:=1200p xlink_mjpeg:=true（2 路 bgr8 约 415 MB/s，超过 USB3
  实测 ~273 MB/s，必须设备端 MJPEG）。

上面参数在本文件声明默认值，其余参数（曝光、录制、foxglove 等）与 oak.launch.py 相同，
全部可在命令行覆盖（如 cam_board_sockets:="[CAM_A, CAM_B, CAM_C, CAM_D]"）。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

AR0234_DEFAULTS = {
    "camera_name": "ar0234",
    "camera_type": "color",
    "resolution": "400p",
    "fps": "30",
}


def generate_launch_description():
    oak_launch = os.path.join(os.path.dirname(os.path.abspath(__file__)), "oak.launch.py")
    # 先在本层声明默认值，再以 LaunchConfiguration 传给被 include 的 launch：
    # 直接传字面值会在 include 时覆盖掉命令行传入的值。
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in AR0234_DEFAULTS.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(oak_launch),
            launch_arguments={name: LaunchConfiguration(name) for name in AR0234_DEFAULTS}.items(),
        ),
    ])
