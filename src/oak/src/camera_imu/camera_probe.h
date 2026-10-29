#pragma once

#include <chrono>
#include <string>
#include <vector>

#include "camera_imu/config.h"
#include "depthai/depthai.hpp"

namespace oak_ffc_camera_imu {

// 固件上报有 sensor 的插座名（getConnectedCameras，只认 CAM_A..D）。
// 注意 depthai 2.x 固件在 FFC-4P 上会误报：只插 A/B 的 AR0234 也报出 CAM_C，
// 那一路 I2C 有应答但 MIPI 不出帧，所以这里的结果只能当候选，要再用 probeStreamingSockets 确认。
std::vector<std::string> reportedSockets(dai::Device& device);

// 在刚启动、尚未跑 pipeline 的 device 上起一个探测 pipeline：每路相机直连 XLinkOut（不经 Sync），
// 返回 timeout 内真正出过帧的插座（保持 candidates 顺序）。
// 这会占用 device 的这一次启动（v2 一次启动只能 startPipeline 一次），调用方之后要重开设备。
std::vector<std::string> probeStreamingSockets(
    dai::Device& device,
    const DriverConfig& config,
    const std::vector<std::string>& candidates,
    std::chrono::milliseconds timeout);

}  // namespace oak_ffc_camera_imu
