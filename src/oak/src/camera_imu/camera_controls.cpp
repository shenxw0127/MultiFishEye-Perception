#include "camera_imu/camera_controls.h"

#include <set>

namespace oak_ffc_camera_imu {

dai::CameraControl makeCameraControl(const DriverConfig& config) {
  dai::CameraControl control;
  applyCameraControls(control, config);
  return control;
}

void applyCameraControls(dai::CameraControl& control, const DriverConfig& config) {
  if (config.manual_exposure_us > 0 && config.manual_iso > 0) {
    control.setManualExposure(
        static_cast<uint32_t>(config.manual_exposure_us),
        static_cast<uint32_t>(config.manual_iso));
  } else {
    control.setAutoExposureEnable();
    if (config.exposure_compensation != 0) {
      control.setAutoExposureCompensation(config.exposure_compensation);
    }
  }

  if (config.manual_wb_kelvin > 0) {
    control.setManualWhiteBalance(config.manual_wb_kelvin);
  } else {
    control.setAutoWhiteBalanceMode(dai::CameraControl::AutoWhiteBalanceMode::AUTO);
  }

  if (config.manual_focus >= 0) {
    control.setManualFocus(static_cast<uint8_t>(config.manual_focus));
  }

  control.setBrightness(config.brightness);
  control.setContrast(config.contrast);
  control.setSaturation(config.saturation);
  control.setSharpness(config.sharpness);
  control.setLumaDenoise(config.luma_denoise);
  control.setChromaDenoise(config.chroma_denoise);
}

bool isCameraControlParameter(const std::string& name) {
  static const std::set<std::string> parameter_names = {
      "manual_exposure_us",
      "manual_iso",
      "exposure_compensation",
      "manual_wb_kelvin",
      "brightness",
      "contrast",
      "saturation",
      "sharpness",
      "luma_denoise",
      "chroma_denoise",
      "manual_focus",
  };
  return parameter_names.find(name) != parameter_names.end();
}

}  // namespace oak_ffc_camera_imu
