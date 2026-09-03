#include "camera_imu/component.h"

#include <cstdlib>
#include <stdexcept>

#include "camera_imu/camera_controls.h"
#include "camera_imu/imu_correction.h"
#include "rclcpp_components/register_node_macro.hpp"

namespace oak_ffc_camera_imu {

OakFfcCameraImuComponent::OakFfcCameraImuComponent(const rclcpp::NodeOptions& options)
    : rclcpp::Node("oak_ffc_camera_imu", options) {
  DriverConfig defaults;
  declareParameters(*this, defaults);
  config_ = readParameters(*this);

  parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&OakFfcCameraImuComponent::onParametersChanged, this, std::placeholders::_1));

  startDevice();
}

OakFfcCameraImuComponent::~OakFfcCameraImuComponent() {
  image_publisher_.reset();
  imu_publisher_.reset();
  device_.reset();
}

void OakFfcCameraImuComponent::startDevice() {
  if (!config_.oak_fw_uri.empty()) {
    if (setenv("DEPTHAI_DEVICE_BINARY", config_.oak_fw_uri.c_str(), 1) != 0) {
      throw std::runtime_error("failed to set DEPTHAI_DEVICE_BINARY");
    }
    RCLCPP_INFO(get_logger(), "Using OAK firmware: %s", config_.oak_fw_uri.c_str());
  }

  pipeline_bundle_ = createPipeline(config_);
  RCLCPP_INFO(
      get_logger(),
      "Starting OAK FFC camera/IMU component: camera=%s type=%s resolution=%s fps=%d imu_hz=%d compressed=%s",
      config_.camera_name.c_str(),
      toString(config_.camera_type).c_str(),
      config_.resolution.c_str(),
      config_.fps,
      config_.imu_hz,
      config_.compressed ? "true" : "false");
  if (config_.manual_exposure_us > 0) {
    RCLCPP_INFO(
        get_logger(),
        "Using manual camera exposure: exposure=%dus iso=%d",
        config_.manual_exposure_us,
        config_.manual_iso);
  }
  if (config_.manual_wb_kelvin > 0) {
    RCLCPP_INFO(get_logger(), "Using manual white balance: %dK", config_.manual_wb_kelvin);
  }

  device_ = std::make_unique<dai::Device>(pipeline_bundle_.pipeline);
  openCameraControlQueues();

  auto sync_queue = device_->getOutputQueue("sync", config_.image_queue_size, false);
  image_publisher_ = std::make_unique<SyncedImagePublisher>(
      *this,
      sync_queue,
      pipeline_bundle_.image_topics,
      config_.tf_prefix,
      config_.compressed,
      config_.lazy_publisher);
  image_publisher_->start();

  auto imu_queue = device_->getOutputQueue("imu", config_.imu_queue_size, false);
  auto imu_correction = ImuCorrection::loadFromYaml(config_.imu_config_path);
  if (imu_correction.enabled()) {
    RCLCPP_INFO(get_logger(), "Using IMU correction config: %s", config_.imu_config_path.c_str());
  }
  imu_publisher_ = std::make_unique<ImuPublisher>(
      *this,
      imu_queue,
      "imu",
      config_.tf_prefix + "_imu_frame",
      imu_correction);
  imu_publisher_->start();
}

void OakFfcCameraImuComponent::openCameraControlQueues() {
  camera_control_queues_.clear();
  for (const auto& stream : pipeline_bundle_.control_streams) {
    camera_control_queues_[stream.first] = device_->getInputQueue(stream.second, 4, false);
  }
}

void OakFfcCameraImuComponent::sendCameraControls(const DriverConfig& config) {
  if (camera_control_queues_.empty()) {
    return;
  }

  auto control = makeCameraControl(config);
  for (const auto& queue : camera_control_queues_) {
    queue.second->send(control);
  }
}

rcl_interfaces::msg::SetParametersResult OakFfcCameraImuComponent::onParametersChanged(
    const std::vector<rclcpp::Parameter>& parameters) {
  auto updated = config_;
  bool camera_controls_changed = false;
  bool pipeline_parameters_changed = false;

  try {
    for (const auto& parameter : parameters) {
      const auto& name = parameter.get_name();
      camera_controls_changed = camera_controls_changed || isCameraControlParameter(name);
      pipeline_parameters_changed = pipeline_parameters_changed || !isCameraControlParameter(name);
      if (name == "tf_prefix") {
        updated.tf_prefix = parameter.as_string();
      } else if (name == "camera_name") {
        updated.camera_name = parameter.as_string();
      } else if (name == "camera_type") {
        updated.camera_type = cameraTypeFromString(parameter.as_string());
      } else if (name == "resolution") {
        updated.resolution = parameter.as_string();
      } else if (name == "fps") {
        updated.fps = static_cast<int>(parameter.as_int());
      } else if (name == "compressed") {
        updated.compressed = parameter.as_bool();
      } else if (name == "oak_fw_uri") {
        updated.oak_fw_uri = parameter.as_string();
      } else if (name == "imu_config_path") {
        updated.imu_config_path = parameter.as_string();
      } else if (name == "imu_hz") {
        updated.imu_hz = static_cast<int>(parameter.as_int());
      } else if (name == "cam_board_sockets") {
        updated.cam_board_sockets = parseSocketList(parameter);
      } else if (name == "sync_threshold_ms") {
        updated.sync_threshold_ms = static_cast<int>(parameter.as_int());
      } else if (name == "image_queue_size") {
        updated.image_queue_size = static_cast<int>(parameter.as_int());
      } else if (name == "imu_queue_size") {
        updated.imu_queue_size = static_cast<int>(parameter.as_int());
      } else if (name == "lazy_publisher") {
        updated.lazy_publisher = parameter.as_bool();
      } else if (name == "manual_exposure_us") {
        updated.manual_exposure_us = static_cast<int>(parameter.as_int());
      } else if (name == "manual_iso") {
        updated.manual_iso = static_cast<int>(parameter.as_int());
      } else if (name == "exposure_compensation") {
        updated.exposure_compensation = static_cast<int>(parameter.as_int());
      } else if (name == "manual_wb_kelvin") {
        updated.manual_wb_kelvin = static_cast<int>(parameter.as_int());
      } else if (name == "brightness") {
        updated.brightness = static_cast<int>(parameter.as_int());
      } else if (name == "contrast") {
        updated.contrast = static_cast<int>(parameter.as_int());
      } else if (name == "saturation") {
        updated.saturation = static_cast<int>(parameter.as_int());
      } else if (name == "sharpness") {
        updated.sharpness = static_cast<int>(parameter.as_int());
      } else if (name == "luma_denoise") {
        updated.luma_denoise = static_cast<int>(parameter.as_int());
      } else if (name == "chroma_denoise") {
        updated.chroma_denoise = static_cast<int>(parameter.as_int());
      } else if (name == "manual_focus") {
        updated.manual_focus = static_cast<int>(parameter.as_int());
      }
    }
    validateConfig(updated);
  } catch (const std::exception& error) {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = false;
    result.reason = error.what();
    return result;
  }

  if (camera_controls_changed) {
    try {
      sendCameraControls(updated);
      RCLCPP_INFO(get_logger(), "Applied updated camera controls to the running OAK pipeline.");
    } catch (const std::exception& error) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = false;
      result.reason = error.what();
      return result;
    }
  }

  config_ = updated;

  if (pipeline_parameters_changed) {
    RCLCPP_WARN(
        get_logger(),
        "OAK pipeline parameter was updated in ROS. Restart this node for hardware pipeline changes to take effect.");
  }

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  return result;
}

}  // namespace oak_ffc_camera_imu

RCLCPP_COMPONENTS_REGISTER_NODE(oak_ffc_camera_imu::OakFfcCameraImuComponent)
