#include "camera_imu/component.h"

#include <cstdlib>
#include <stdexcept>

#include "camera_imu/camera_controls.h"
#include "camera_imu/imu_correction.h"
#include "rclcpp_components/register_node_macro.hpp"

namespace oak_ffc_camera_imu {
namespace {

constexpr auto kHealthPollInterval = std::chrono::milliseconds(200);

}  // namespace

OakFfcCameraImuComponent::OakFfcCameraImuComponent(const rclcpp::NodeOptions& options)
    : rclcpp::Node("oak_ffc_camera_imu", options) {
  DriverConfig defaults;
  declareParameters(*this, defaults);
  config_ = readParameters(*this);

  if (!config_.oak_fw_uri.empty()) {
    if (setenv("DEPTHAI_DEVICE_BINARY", config_.oak_fw_uri.c_str(), 1) != 0) {
      throw std::runtime_error("failed to set DEPTHAI_DEVICE_BINARY");
    }
    RCLCPP_INFO(get_logger(), "Using OAK firmware: %s", config_.oak_fw_uri.c_str());
  }

  RCLCPP_INFO(
      get_logger(),
      "Starting OAK FFC camera/IMU component: camera=%s type=%s resolution=%s fps=%d imu_hz=%d compressed=%s",
      config_.camera_name.c_str(),
      toString(config_.camera_type).c_str(),
      config_.resolution.c_str(),
      config_.fps,
      config_.imu_hz,
      config_.compressed ? "true" : "false");
  if (config_.manual_exposure_us > 0 && config_.manual_iso > 0) {
    RCLCPP_INFO(
        get_logger(),
        "Using manual camera exposure: exposure=%dus iso=%d",
        config_.manual_exposure_us,
        config_.manual_iso);
  } else if (config_.auto_exposure_limit_us > 0) {
    RCLCPP_INFO(
        get_logger(),
        "Using auto exposure capped at %dus; gain is left to auto.",
        config_.auto_exposure_limit_us);
  }
  if (config_.manual_wb_kelvin > 0) {
    RCLCPP_INFO(get_logger(), "Using manual white balance: %dK", config_.manual_wb_kelvin);
  }

  // Shared DepthAI->ROS time base so image and IMU streams are aligned. Kept for
  // the node lifetime: DepthAI stamps are host-synced steady clock, so reusing
  // the same base across reconnects keeps stamps monotonic.
  time_converter_ = std::make_shared<DaiTimeConverter>();

  image_publisher_ = std::make_unique<SyncedImagePublisher>(
      *this,
      imageTopics(config_),
      config_.tf_prefix,
      config_.compressed,
      config_.lazy_publisher,
      time_converter_);

  auto imu_correction = config_.imu_correction_enabled
                            ? ImuCorrection::loadFromYaml(config_.imu_config_path)
                            : ImuCorrection::identity();
  if (imu_correction.enabled()) {
    RCLCPP_INFO(get_logger(), "Using IMU correction config: %s", config_.imu_config_path.c_str());
  } else {
    RCLCPP_INFO(get_logger(), "IMU correction disabled; publishing raw IMU data.");
  }
  imu_publisher_ = std::make_unique<ImuPublisher>(
      *this,
      "imu",
      config_.tf_prefix + "_imu_frame",
      imu_correction,
      time_converter_);

  parameter_callback_handle_ = add_on_set_parameters_callback(
      std::bind(&OakFfcCameraImuComponent::onParametersChanged, this, std::placeholders::_1));

  // 设备连接放到后台：没插相机时组件照常加载，插上后自动连上。
  supervisor_thread_ = std::thread(&OakFfcCameraImuComponent::superviseDevice, this);
}

OakFfcCameraImuComponent::~OakFfcCameraImuComponent() {
  running_ = false;
  wake_cv_.notify_all();
  if (supervisor_thread_.joinable()) {
    supervisor_thread_.join();
  }
  image_publisher_.reset();
  imu_publisher_.reset();
}

void OakFfcCameraImuComponent::superviseDevice() {
  int reconnect_interval_ms = 0;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    reconnect_interval_ms = config_.reconnect_interval_ms;
  }
  const auto reconnect_interval = std::chrono::milliseconds(reconnect_interval_ms);

  while (running_ && rclcpp::ok()) {
    try {
      connectDevice();
    } catch (const std::exception& error) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 10000, "OAK not available, retrying: %s", error.what());
      disconnectDevice();
      if (!waitFor(reconnect_interval)) {
        break;
      }
      continue;
    }

    RCLCPP_INFO(get_logger(), "OAK connected: %s", device_->getMxId().c_str());
    while (running_ && rclcpp::ok() && deviceHealthy()) {
      waitFor(kHealthPollInterval);
    }

    if (running_ && rclcpp::ok()) {
      RCLCPP_WARN(get_logger(), "OAK disconnected, reconnecting...");
    }
    disconnectDevice();
    if (!waitFor(reconnect_interval)) {
      break;
    }
  }
  disconnectDevice();
}

void OakFfcCameraImuComponent::connectDevice() {
  DriverConfig config;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config = config_;
  }

  auto bundle = createPipeline(config);
  // 构造会阻塞到找到设备并上传完 pipeline；设备不存在时抛异常，由 supervisor 重试。
  device_ = std::make_unique<dai::Device>(bundle.pipeline);

  {
    std::lock_guard<std::mutex> lock(control_mutex_);
    camera_control_queues_.clear();
    for (const auto& stream : bundle.control_streams) {
      camera_control_queues_[stream.first] = device_->getInputQueue(stream.second, 4, false);
    }
  }

  image_publisher_->attach(device_->getOutputQueue("sync", config.image_queue_size, false));
  imu_publisher_->attach(device_->getOutputQueue("imu", config.imu_queue_size, false));
}

void OakFfcCameraImuComponent::disconnectDevice() {
  // 先停读线程，再关设备，避免读线程访问已销毁的队列。
  image_publisher_->detach();
  imu_publisher_->detach();
  {
    std::lock_guard<std::mutex> lock(control_mutex_);
    camera_control_queues_.clear();
  }
  try {
    device_.reset();
  } catch (const std::exception& error) {
    RCLCPP_WARN(get_logger(), "Error while closing OAK device: %s", error.what());
  }
}

bool OakFfcCameraImuComponent::deviceHealthy() const {
  return device_ && !device_->isClosed() && image_publisher_->healthy() && imu_publisher_->healthy();
}

bool OakFfcCameraImuComponent::waitFor(std::chrono::milliseconds duration) {
  std::unique_lock<std::mutex> lock(wake_mutex_);
  wake_cv_.wait_for(lock, duration, [this] { return !running_; });
  return running_;
}

void OakFfcCameraImuComponent::sendCameraControls(const DriverConfig& config) {
  std::lock_guard<std::mutex> lock(control_mutex_);
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
  rcl_interfaces::msg::SetParametersResult result;
  DriverConfig updated;
  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    updated = config_;
  }

  try {
    for (const auto& parameter : parameters) {
      const auto& name = parameter.get_name();
      if (!isCameraControlParameter(name)) {
        // pipeline 拓扑参数运行中不能改：publisher 的 topic 是启动时定的，
        // 若放进 config_，下次重连会建出与 publisher 不匹配的 pipeline。
        result.successful = false;
        result.reason = name + " is a pipeline parameter; restart the node to change it";
        return result;
      }
      if (name == "manual_exposure_us") {
        updated.manual_exposure_us = static_cast<int>(parameter.as_int());
      } else if (name == "manual_iso") {
        updated.manual_iso = static_cast<int>(parameter.as_int());
      } else if (name == "exposure_compensation") {
        updated.exposure_compensation = static_cast<int>(parameter.as_int());
      } else if (name == "auto_exposure_limit_us") {
        updated.auto_exposure_limit_us = static_cast<int>(parameter.as_int());
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
    result.successful = false;
    result.reason = error.what();
    return result;
  }

  // 断连时没有 control queue（或发送途中设备掉线），参数照样接受：
  // 新值随 config_ 在重连时经 initialControl 生效。
  try {
    sendCameraControls(updated);
  } catch (const std::exception& error) {
    RCLCPP_WARN(get_logger(), "Camera controls will apply on reconnect: %s", error.what());
  }

  {
    std::lock_guard<std::mutex> lock(config_mutex_);
    config_ = updated;
  }
  RCLCPP_INFO(get_logger(), "Applied updated camera controls.");
  result.successful = true;
  return result;
}

}  // namespace oak_ffc_camera_imu

RCLCPP_COMPONENTS_REGISTER_NODE(oak_ffc_camera_imu::OakFfcCameraImuComponent)
