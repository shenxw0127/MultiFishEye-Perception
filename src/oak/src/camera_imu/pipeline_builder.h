#pragma once

#include <map>
#include <string>

#include "depthai/depthai.hpp"
#include "camera_imu/config.h"

namespace oak_ffc_camera_imu {

struct PipelineBundle {
  dai::Pipeline pipeline;
  int width = 0;
  int height = 0;
  std::map<std::string, std::string> image_topics;
  std::map<std::string, std::string> control_streams;
};

// config.cam_board_sockets 必须是已和设备检测结果取过交集的显式列表（不能是 ALL）。
PipelineBundle createPipeline(const DriverConfig& config);

// 设备启动配置（目前只有 OV9782 的 FSIN GPIO）。只取决于 sensor 型号、与插座无关，
// 所以可以先用它启动设备、查到插着哪些相机，再按结果建 pipeline。
dai::Device::Config deviceConfig(const DriverConfig& config);

// socket 名 -> ROS 图像 topic。发布端在连设备之前就要建好 publisher，所以单独暴露。
// ALL 模式下为全部插座建 topic，实际只发检测到的那几路。
std::map<std::string, std::string> imageTopics(const DriverConfig& config);

}  // namespace oak_ffc_camera_imu
