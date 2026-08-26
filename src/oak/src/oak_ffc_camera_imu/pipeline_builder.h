#pragma once

#include <map>
#include <string>

#include "depthai/depthai.hpp"
#include "oak_ffc_camera_imu/config.h"

namespace oak_ffc_camera_imu {

struct PipelineBundle {
  dai::Pipeline pipeline;
  int width = 0;
  int height = 0;
  std::map<std::string, std::string> image_topics;
  std::map<std::string, std::string> control_streams;
};

PipelineBundle createPipeline(const DriverConfig& config);

}  // namespace oak_ffc_camera_imu
