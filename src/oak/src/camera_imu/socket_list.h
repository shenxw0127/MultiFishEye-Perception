#pragma once

#include <string>
#include <vector>

#include "rclcpp/node.hpp"

namespace oak_ffc_camera_imu {

// cam_board_sockets 写成 "ALL"（或 [ALL]）时，连设备后按实际插着的相机自动选路。
constexpr const char* kAllSockets = "ALL";

// FFC-4P 全部插座，也是 ALL 模式下预先建 publisher / 录制订阅的话题集合。
const std::vector<std::string>& allSocketNames();

// 驱动和录制组件共用：cam_board_sockets 允许 launch 传字符串（"ALL" / "CAM_A, CAM_B"）
// 或字符串数组，因此必须以 dynamic_typing 声明，否则类型和默认值不一致时 declare 直接抛异常。
void declareSocketListParameter(rclcpp::Node& node, const std::vector<std::string>& defaults);
std::vector<std::string> parseSocketList(const rclcpp::Parameter& parameter);

bool isAutoDetect(const std::vector<std::string>& sockets);
// ALL 展开成全部插座，否则原样返回。
std::vector<std::string> expandSockets(const std::vector<std::string>& sockets);

struct SocketSelection {
  std::vector<std::string> selected;  // 要建 pipeline 的插座，保持请求顺序
  std::vector<std::string> missing;   // 请求了但设备上没检测到的插座
};

// 请求列表与设备检测结果取交集。缺失的插座若照样进 pipeline，Sync 节点会一直等那一路，
// 整个图像流卡死，所以必须在建 pipeline 前剔除。
SocketSelection selectSockets(const std::vector<std::string>& requested, const std::vector<std::string>& detected);

}  // namespace oak_ffc_camera_imu
