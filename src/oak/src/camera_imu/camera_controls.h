#pragma once

#include <string>

#include "depthai/depthai.hpp"
#include "camera_imu/config.h"

namespace oak_ffc_camera_imu {

dai::CameraControl makeCameraControl(const DriverConfig& config);
void applyCameraControls(dai::CameraControl& control, const DriverConfig& config);

bool isCameraControlParameter(const std::string& name);

}  // namespace oak_ffc_camera_imu
