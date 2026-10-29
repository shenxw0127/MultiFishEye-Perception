"""USB2 + 厂商原生黑白 + 400p@30 的 oak.launch.py 预设。

    ros2 launch oak_ros2 oak_usb2_mono.launch.py
    ros2 launch oak_ros2 oak_usb2_mono.launch.py foxglove:=false

- usb_speed=high：强制协商 USB2，避开 USB3 在 GNSS L1 附近的宽带射频噪声。
- camera_type=mono：MonoCamera 节点，由厂商固件按 MONO 模式出 GRAY8，不做彩色转灰度。
- xlink_mjpeg=true：4×640×400×30 裸 GRAY8 需要 30.7 MB/s，USB2 实测只能跑 ~24fps；
  设备端 MJPEG(q95) 后约 7.5 MB/s，满 30fps 零丢帧，主机解码回 mono8 发布，下游不变。

上面 5 个参数在本文件声明默认值，其余参数（曝光、录制、foxglove 等）与 oak.launch.py 相同，
全部可在命令行覆盖（如 fps:=15）。
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

USB2_MONO_DEFAULTS = {
    "camera_type": "mono",
    "resolution": "400p",
    "fps": "30",
    "usb_speed": "high",
    "xlink_mjpeg": "true",
}


def generate_launch_description():
    oak_launch = os.path.join(os.path.dirname(os.path.abspath(__file__)), "oak.launch.py")
    # 先在本层声明默认值，再以 LaunchConfiguration 传给被 include 的 launch：
    # 直接传字面值会在 include 时覆盖掉命令行传入的值。
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in USB2_MONO_DEFAULTS.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(oak_launch),
            launch_arguments={name: LaunchConfiguration(name) for name in USB2_MONO_DEFAULTS}.items(),
        ),
    ])
