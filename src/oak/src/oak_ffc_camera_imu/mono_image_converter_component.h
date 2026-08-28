#pragma once

#include <string>

#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace oak_ffc_camera_imu {

class MonoImageConverterComponent : public rclcpp::Node {
 public:
  explicit MonoImageConverterComponent(const rclcpp::NodeOptions& options);

 private:
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  std::string output_frame_id_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

}  // namespace oak_ffc_camera_imu
