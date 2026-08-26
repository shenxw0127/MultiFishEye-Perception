#include "oak_ffc_camera_imu/component.h"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<oak_ffc_camera_imu::OakFfcCameraImuComponent>());
  rclcpp::shutdown();
  return 0;
}
