#include "oak_ffc_camera_imu/config.h"

#include <algorithm>
#include <sstream>
#include <stdexcept>

namespace oak_ffc_camera_imu {
namespace {

std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t\n\r[]");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\n\r[]");
  return value.substr(first, last - first + 1);
}

}  // namespace

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
      {"400p", {640, 400, ColorResolution::THE_720_P, MonoResolution::THE_400_P, false, true}},
      {"480p", {640, 480, ColorResolution::THE_720_P, MonoResolution::THE_480_P, false, true}},
      {"720p", {1280, 720, ColorResolution::THE_720_P, MonoResolution::THE_720_P, true, true}},
      {"800p", {1280, 800, ColorResolution::THE_800_P, MonoResolution::THE_800_P, true, true}},
      {"1080p", {1920, 1080, ColorResolution::THE_1080_P, MonoResolution::THE_1200_P, true, false}},
      {"1200p", {1920, 1200, ColorResolution::THE_1200_P, MonoResolution::THE_1200_P, true, true}},
      {"4k", {3840, 2160, ColorResolution::THE_4_K, MonoResolution::THE_1200_P, true, false}},
  };
  return options;
}

void declareParameters(rclcpp::Node& node, const DriverConfig& defaults) {
  node.declare_parameter("tf_prefix", defaults.tf_prefix);
  node.declare_parameter("camera_name", defaults.camera_name);
  node.declare_parameter("camera_type", toString(defaults.camera_type));
  node.declare_parameter("resolution", defaults.resolution);
  node.declare_parameter("fps", defaults.fps);
  node.declare_parameter("compressed", defaults.compressed);
  node.declare_parameter("oak_fw_uri", defaults.oak_fw_uri);
  node.declare_parameter("imu_hz", defaults.imu_hz);
  node.declare_parameter("cam_board_sockets", defaults.cam_board_sockets);
  node.declare_parameter("sync_threshold_ms", defaults.sync_threshold_ms);
  node.declare_parameter("image_queue_size", defaults.image_queue_size);
  node.declare_parameter("imu_queue_size", defaults.imu_queue_size);
  node.declare_parameter("lazy_publisher", defaults.lazy_publisher);
}

DriverConfig readParameters(rclcpp::Node& node) {
  DriverConfig config;
  config.tf_prefix = node.get_parameter("tf_prefix").as_string();
  config.camera_name = node.get_parameter("camera_name").as_string();
  config.camera_type = cameraTypeFromString(node.get_parameter("camera_type").as_string());
  config.resolution = node.get_parameter("resolution").as_string();
  config.fps = static_cast<int>(node.get_parameter("fps").as_int());
  config.compressed = node.get_parameter("compressed").as_bool();
  config.oak_fw_uri = node.get_parameter("oak_fw_uri").as_string();
  config.imu_hz = static_cast<int>(node.get_parameter("imu_hz").as_int());
  config.cam_board_sockets = parseSocketList(node.get_parameter("cam_board_sockets"));
  config.sync_threshold_ms = static_cast<int>(node.get_parameter("sync_threshold_ms").as_int());
  config.image_queue_size = static_cast<int>(node.get_parameter("image_queue_size").as_int());
  config.imu_queue_size = static_cast<int>(node.get_parameter("imu_queue_size").as_int());
  config.lazy_publisher = node.get_parameter("lazy_publisher").as_bool();
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
  if (config.imu_hz <= 0) {
    throw std::runtime_error("imu_hz must be greater than zero");
  }
  if (config.sync_threshold_ms < 0) {
    throw std::runtime_error("sync_threshold_ms must be non-negative");
  }
  if (config.image_queue_size <= 0 || config.imu_queue_size <= 0) {
    throw std::runtime_error("queue sizes must be greater than zero");
  }

  const auto& sockets = cameraSocketOptions();
  for (const auto& socket_name : config.cam_board_sockets) {
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

std::vector<std::string> parseSocketList(const rclcpp::Parameter& parameter) {
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    return parameter.as_string_array();
  }

  if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
    throw std::runtime_error("cam_board_sockets must be a string array or comma-separated string");
  }

  std::vector<std::string> sockets;
  std::stringstream stream(parameter.as_string());
  std::string item;
  while (std::getline(stream, item, ',')) {
    const auto socket = trim(item);
    if (!socket.empty()) {
      sockets.push_back(socket);
    }
  }
  return sockets;
}

}  // namespace oak_ffc_camera_imu
