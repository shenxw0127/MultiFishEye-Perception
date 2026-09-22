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

PipelineBundle createPipeline(const DriverConfig& config);

// socket 名 -> ROS 图像 topic。发布端在连设备之前就要建好 publisher，所以单独暴露。
std::map<std::string, std::string> imageTopics(const DriverConfig& config);

}  // namespace oak_ffc_camera_imu
