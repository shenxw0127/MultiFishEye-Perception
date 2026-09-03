#pragma once

#include <map>
#include <memory>
#include <string>

#include "depthai/depthai.hpp"
#include "camera_imu/config.h"
#include "camera_imu/image_publisher.h"
#include "camera_imu/imu_publisher.h"
#include "camera_imu/pipeline_builder.h"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

namespace oak_ffc_camera_imu {

class OakFfcCameraImuComponent : public rclcpp::Node {
 public:
  explicit OakFfcCameraImuComponent(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~OakFfcCameraImuComponent() override;

 private:
  void startDevice();
  void openCameraControlQueues();
  void sendCameraControls(const DriverConfig& config);
  rcl_interfaces::msg::SetParametersResult onParametersChanged(const std::vector<rclcpp::Parameter>& parameters);

  DriverConfig config_;
  PipelineBundle pipeline_bundle_;
  std::unique_ptr<dai::Device> device_;
  std::unique_ptr<SyncedImagePublisher> image_publisher_;
  std::unique_ptr<ImuPublisher> imu_publisher_;
  std::map<std::string, std::shared_ptr<dai::DataInputQueue>> camera_control_queues_;
  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace oak_ffc_camera_imu
