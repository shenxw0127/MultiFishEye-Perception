#pragma once

#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "depthai/depthai.hpp"
#include "rclcpp/node.hpp"

namespace oak_ffc_camera_imu {

enum class CameraType {
  Color,
  Mono,
};

struct CameraSocketConfig {
  dai::CameraBoardSocket socket;
  std::string image_topic;
};

struct ResolutionConfig {
  int width = 0;
  int height = 0;
  dai::node::ColorCamera::Properties::SensorResolution color_resolution;
  dai::node::MonoCamera::Properties::SensorResolution mono_resolution;
  bool supports_color = false;
  bool supports_mono = false;
};

struct DriverConfig {
  std::string tf_prefix = "oak";
  std::string camera_name = "ov9782";
  CameraType camera_type = CameraType::Color;
  std::string resolution = "800p";
  int fps = 30;
  bool compressed = true;
  std::string oak_fw_uri;
  int imu_hz = 200;
  std::vector<std::string> cam_board_sockets = {"CAM_A", "CAM_B", "CAM_C", "CAM_D"};
  int sync_threshold_ms = 50;
  int image_queue_size = 1;
  int imu_queue_size = 50;
  bool lazy_publisher = true;
  int manual_exposure_us = 0;
  int manual_iso = 0;
  int exposure_compensation = 0;
  int manual_wb_kelvin = 0;
  int brightness = 0;
  int contrast = 0;
  int saturation = 0;
  int sharpness = 1;
  int luma_denoise = 1;
  int chroma_denoise = 1;
  int manual_focus = -1;
};

const std::map<std::string, CameraSocketConfig>& cameraSocketOptions();
const std::map<std::string, ResolutionConfig>& resolutionOptions();

void declareParameters(rclcpp::Node& node, const DriverConfig& defaults);
DriverConfig readParameters(rclcpp::Node& node);
void validateConfig(const DriverConfig& config);

std::string toString(CameraType type);
CameraType cameraTypeFromString(const std::string& value);
std::vector<std::string> parseSocketList(const rclcpp::Parameter& parameter);

}  // namespace oak_ffc_camera_imu
