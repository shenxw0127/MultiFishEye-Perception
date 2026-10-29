#pragma once

#include <memory>
#include <string>

#include "builtin_interfaces/msg/time.hpp"
#include "depthai/depthai.hpp"
#include "oak_interfaces/msg/isp_meta.hpp"

namespace oak_ffc_camera_imu {

// 把一帧 ImgFrame 自带的 ISP 元数据（固件 3A 当前实际值）转成 ROS 消息。
// stamp 由调用方传入：和图像消息走同一个 DaiTimeConverter，保证与 /<camera>/image 的 header.stamp 完全一致。
oak_interfaces::msg::IspMeta toIspMeta(
    const std::string& camera_name,
    const dai::ImgFrame& frame,
    const builtin_interfaces::msg::Time& stamp);

// 该帧有没有带 ISP 元数据：曝光时间为 0 的真实帧不存在，为 0 说明这条通路上固件没有填元数据。
bool hasIspMeta(const oak_interfaces::msg::IspMeta& meta);

}  // namespace oak_ffc_camera_imu
