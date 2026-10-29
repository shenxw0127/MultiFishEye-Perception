#include "camera_imu/config.h"

#include <stdexcept>

namespace oak_ffc_camera_imu {

const std::map<std::string, CameraSocketConfig>& cameraSocketOptions() {
  static const std::map<std::string, CameraSocketConfig> options = {
      {"CAM_A", {dai::CameraBoardSocket::CAM_A, "CAM_A/image"}},
      {"CAM_B", {dai::CameraBoardSocket::CAM_B, "CAM_B/image"}},
      {"CAM_C", {dai::CameraBoardSocket::CAM_C, "CAM_C/image"}},
      {"CAM_D", {dai::CameraBoardSocket::CAM_D, "CAM_D/image"}},
  };
  return options;
}

const std::map<std::string, ResolutionConfig>& resolutionOptions() {
  using ColorResolution = dai::node::ColorCamera::Properties::SensorResolution;
  using MonoResolution = dai::node::MonoCamera::Properties::SensorResolution;

  static const std::map<std::string, ResolutionConfig> options = {
      // 400p 在 color 下是 800p sensor + ISP 1/2 缩放（1280x800 -> 640x400），保 FOV。
      {"400p", {640, 400, ColorResolution::THE_800_P, MonoResolution::THE_400_P, true, true, 1, 2}},
      {"480p", {640, 480, ColorResolution::THE_720_P, MonoResolution::THE_480_P, false, true}},
      {"720p", {1280, 720, ColorResolution::THE_720_P, MonoResolution::THE_720_P, true, true}},
      {"800p", {1280, 800, ColorResolution::THE_800_P, MonoResolution::THE_800_P, true, true}},
      {"1080p", {1920, 1080, ColorResolution::THE_1080_P, MonoResolution::THE_1200_P, true, false}},
      {"1200p", {1920, 1200, ColorResolution::THE_1200_P, MonoResolution::THE_1200_P, true, true}},
      {"4k", {3840, 2160, ColorResolution::THE_4_K, MonoResolution::THE_1200_P, true, false}},
  };
  return options;
}

ResolutionConfig resolveResolution(const DriverConfig& config) {
  auto resolution = resolutionOptions().at(config.resolution);
  if (config.camera_name == "ar0234") {
    if (config.resolution == "400p") {
      resolution.color_resolution = dai::node::ColorCamera::Properties::SensorResolution::THE_1200_P;
      resolution.isp_scale_num = 1;
      resolution.isp_scale_den = 3;
    } else if (config.resolution == "800p") {
      resolution.color_resolution = dai::node::ColorCamera::Properties::SensorResolution::THE_1200_P;
      resolution.isp_scale_num = 2;
      resolution.isp_scale_den = 3;
    }
  }
  return resolution;
}

void declareParameters(rclcpp::Node& node, const DriverConfig& defaults) {
  node.declare_parameter("tf_prefix", defaults.tf_prefix);
  node.declare_parameter("camera_name", defaults.camera_name);
  node.declare_parameter("camera_type", toString(defaults.camera_type));
  node.declare_parameter("resolution", defaults.resolution);
  node.declare_parameter("fps", defaults.fps);
  node.declare_parameter("compressed", defaults.compressed);
  node.declare_parameter("usb_speed", defaults.usb_speed);
  node.declare_parameter("xlink_mjpeg", defaults.xlink_mjpeg);
  node.declare_parameter("jpeg_quality", defaults.jpeg_quality);
  node.declare_parameter("oak_fw_uri", defaults.oak_fw_uri);
  node.declare_parameter("imu_config_path", defaults.imu_config_path);
  node.declare_parameter("imu_correction_enabled", defaults.imu_correction_enabled);
  node.declare_parameter("imu_hz", defaults.imu_hz);
  declareSocketListParameter(node, defaults.cam_board_sockets);
  node.declare_parameter("sync_threshold_ms", defaults.sync_threshold_ms);
  node.declare_parameter("image_queue_size", defaults.image_queue_size);
  node.declare_parameter("imu_queue_size", defaults.imu_queue_size);
  node.declare_parameter("lazy_publisher", defaults.lazy_publisher);
  node.declare_parameter("reconnect_interval_ms", defaults.reconnect_interval_ms);
  node.declare_parameter("fps_display", defaults.fps_display);
  node.declare_parameter("manual_exposure_us", defaults.manual_exposure_us);
  node.declare_parameter("manual_iso", defaults.manual_iso);
  node.declare_parameter("exposure_compensation", defaults.exposure_compensation);
  node.declare_parameter("auto_exposure_limit_us", defaults.auto_exposure_limit_us);
  node.declare_parameter("manual_wb_kelvin", defaults.manual_wb_kelvin);
  node.declare_parameter("brightness", defaults.brightness);
  node.declare_parameter("contrast", defaults.contrast);
  node.declare_parameter("saturation", defaults.saturation);
  node.declare_parameter("sharpness", defaults.sharpness);
  node.declare_parameter("luma_denoise", defaults.luma_denoise);
  node.declare_parameter("chroma_denoise", defaults.chroma_denoise);
  node.declare_parameter("manual_focus", defaults.manual_focus);
}

DriverConfig readParameters(rclcpp::Node& node) {
  DriverConfig config;
  config.tf_prefix = node.get_parameter("tf_prefix").as_string();
  config.camera_name = node.get_parameter("camera_name").as_string();
  config.camera_type = cameraTypeFromString(node.get_parameter("camera_type").as_string());
  config.resolution = node.get_parameter("resolution").as_string();
  config.fps = static_cast<int>(node.get_parameter("fps").as_int());
  config.compressed = node.get_parameter("compressed").as_bool();
  config.usb_speed = node.get_parameter("usb_speed").as_string();
  config.xlink_mjpeg = node.get_parameter("xlink_mjpeg").as_bool();
  config.jpeg_quality = static_cast<int>(node.get_parameter("jpeg_quality").as_int());
  config.oak_fw_uri = node.get_parameter("oak_fw_uri").as_string();
  config.imu_config_path = node.get_parameter("imu_config_path").as_string();
  config.imu_correction_enabled = node.get_parameter("imu_correction_enabled").as_bool();
  config.imu_hz = static_cast<int>(node.get_parameter("imu_hz").as_int());
  config.cam_board_sockets = parseSocketList(node.get_parameter("cam_board_sockets"));
  config.sync_threshold_ms = static_cast<int>(node.get_parameter("sync_threshold_ms").as_int());
  config.image_queue_size = static_cast<int>(node.get_parameter("image_queue_size").as_int());
  config.imu_queue_size = static_cast<int>(node.get_parameter("imu_queue_size").as_int());
  config.lazy_publisher = node.get_parameter("lazy_publisher").as_bool();
  config.reconnect_interval_ms = static_cast<int>(node.get_parameter("reconnect_interval_ms").as_int());
  config.fps_display = node.get_parameter("fps_display").as_bool();
  config.manual_exposure_us = static_cast<int>(node.get_parameter("manual_exposure_us").as_int());
  config.manual_iso = static_cast<int>(node.get_parameter("manual_iso").as_int());
  config.exposure_compensation = static_cast<int>(node.get_parameter("exposure_compensation").as_int());
  config.auto_exposure_limit_us = static_cast<int>(node.get_parameter("auto_exposure_limit_us").as_int());
  config.manual_wb_kelvin = static_cast<int>(node.get_parameter("manual_wb_kelvin").as_int());
  config.brightness = static_cast<int>(node.get_parameter("brightness").as_int());
  config.contrast = static_cast<int>(node.get_parameter("contrast").as_int());
  config.saturation = static_cast<int>(node.get_parameter("saturation").as_int());
  config.sharpness = static_cast<int>(node.get_parameter("sharpness").as_int());
  config.luma_denoise = static_cast<int>(node.get_parameter("luma_denoise").as_int());
  config.chroma_denoise = static_cast<int>(node.get_parameter("chroma_denoise").as_int());
  config.manual_focus = static_cast<int>(node.get_parameter("manual_focus").as_int());
  validateConfig(config);
  return config;
}

void validateConfig(const DriverConfig& config) {
  if (config.cam_board_sockets.empty()) {
    throw std::runtime_error("cam_board_sockets must contain at least one camera socket");
  }
  if (config.fps <= 0) {
    throw std::runtime_error("fps must be greater than zero");
  }
  usbSpeedFromString(config.usb_speed);
  if (config.jpeg_quality < 1 || config.jpeg_quality > 100) {
    throw std::runtime_error("jpeg_quality must be within 1..100");
  }
  if (config.imu_hz <= 0) {
    throw std::runtime_error("imu_hz must be greater than zero");
  }
  if (config.sync_threshold_ms < 0) {
    throw std::runtime_error("sync_threshold_ms must be non-negative");
  }
  if (config.image_queue_size <= 0 || config.imu_queue_size <= 0) {
    throw std::runtime_error("queue sizes must be greater than zero");
  }
  if (config.reconnect_interval_ms < 100) {
    throw std::runtime_error("reconnect_interval_ms must be at least 100");
  }
  if (config.manual_exposure_us < 0) {
    throw std::runtime_error("manual_exposure_us must be non-negative");
  }
  if (config.manual_iso != 0 && (config.manual_iso < 100 || config.manual_iso > 1600)) {
    throw std::runtime_error("manual_iso must be 0 or within 100..1600");
  }
  if (config.exposure_compensation < -9 || config.exposure_compensation > 9) {
    throw std::runtime_error("exposure_compensation must be within -9..9");
  }
  if (config.auto_exposure_limit_us != 0 &&
      (config.auto_exposure_limit_us < 100 || config.auto_exposure_limit_us > 100000)) {
    throw std::runtime_error("auto_exposure_limit_us must be 0 or within 100..100000");
  }
  if (config.manual_wb_kelvin != 0 &&
      (config.manual_wb_kelvin < 1000 || config.manual_wb_kelvin > 12000)) {
    throw std::runtime_error("manual_wb_kelvin must be 0 or within 1000..12000");
  }
  if (config.brightness < -10 || config.brightness > 10) {
    throw std::runtime_error("brightness must be within -10..10");
  }
  if (config.contrast < -10 || config.contrast > 10) {
    throw std::runtime_error("contrast must be within -10..10");
  }
  if (config.saturation < -10 || config.saturation > 10) {
    throw std::runtime_error("saturation must be within -10..10");
  }
  if (config.sharpness < 0 || config.sharpness > 4) {
    throw std::runtime_error("sharpness must be within 0..4");
  }
  if (config.luma_denoise < 0 || config.luma_denoise > 4) {
    throw std::runtime_error("luma_denoise must be within 0..4");
  }
  if (config.chroma_denoise < 0 || config.chroma_denoise > 4) {
    throw std::runtime_error("chroma_denoise must be within 0..4");
  }
  if (config.manual_focus < -1 || config.manual_focus > 255) {
    throw std::runtime_error("manual_focus must be -1 or within 0..255");
  }

  const auto& sockets = cameraSocketOptions();
  for (const auto& socket_name : expandSockets(config.cam_board_sockets)) {
    if (sockets.find(socket_name) == sockets.end()) {
      throw std::runtime_error("unsupported camera socket: " + socket_name);
    }
  }

  const auto& resolutions = resolutionOptions();
  const auto resolution_it = resolutions.find(config.resolution);
  if (resolution_it == resolutions.end()) {
    throw std::runtime_error("unsupported resolution: " + config.resolution);
  }

  const auto& resolution = resolution_it->second;
  if (config.camera_type == CameraType::Color && !resolution.supports_color) {
    throw std::runtime_error("resolution " + config.resolution + " is not supported by color cameras");
  }
  if (config.camera_type == CameraType::Mono && !resolution.supports_mono) {
    throw std::runtime_error("resolution " + config.resolution + " is not supported by mono cameras");
  }
}

dai::UsbSpeed usbSpeedFromString(const std::string& value) {
  if (value == "super") {
    return dai::UsbSpeed::SUPER;
  }
  if (value == "high") {
    return dai::UsbSpeed::HIGH;
  }
  throw std::runtime_error("usb_speed must be either 'super' or 'high'");
}

std::string toString(CameraType type) {
  return type == CameraType::Color ? "color" : "mono";
}

CameraType cameraTypeFromString(const std::string& value) {
  if (value == "color") {
    return CameraType::Color;
  }
  if (value == "mono") {
    return CameraType::Mono;
  }
  throw std::runtime_error("camera_type must be either 'color' or 'mono'");
}

}  // namespace oak_ffc_camera_imu
