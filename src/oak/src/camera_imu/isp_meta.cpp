#include "camera_imu/isp_meta.h"

#include <chrono>

namespace oak_ffc_camera_imu {

oak_interfaces::msg::IspMeta toIspMeta(
    const std::string& camera_name,
    const dai::ImgFrame& frame,
    const builtin_interfaces::msg::Time& stamp) {
  oak_interfaces::msg::IspMeta meta;
  meta.camera = camera_name;
  meta.stamp = stamp;
  meta.sequence_num = frame.getSequenceNum();
  const auto exposure_us = frame.getExposureTime().count();
  meta.exposure_time_us = exposure_us > 0 ? static_cast<uint32_t>(exposure_us) : 0u;
  meta.iso = frame.getSensitivity();
  meta.color_temperature_k = frame.getColorTemperature();
  meta.lens_position = frame.getLensPosition();
  return meta;
}

bool hasIspMeta(const oak_interfaces::msg::IspMeta& meta) {
  return meta.exposure_time_us > 0;
}

}  // namespace oak_ffc_camera_imu
