#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "depthai/depthai.hpp"
#include "oak_ffc_camera_imu/imu_correction.h"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace oak_ffc_camera_imu {

class ImuPublisher {
 public:
  ImuPublisher(
      rclcpp::Node& node,
      std::shared_ptr<dai::DataOutputQueue> queue,
      std::string topic,
      std::string frame_id,
      ImuCorrection correction);
  ~ImuPublisher();

  ImuPublisher(const ImuPublisher&) = delete;
  ImuPublisher& operator=(const ImuPublisher&) = delete;

  void start();
  void stop();

 private:
  void run();
  sensor_msgs::msg::Imu toRosMsg(const dai::IMUPacket& packet) const;

  rclcpp::Node& node_;
  std::shared_ptr<dai::DataOutputQueue> queue_;
  std::string frame_id_;
  ImuCorrection correction_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  std::atomic_bool running_{false};
  std::thread thread_;
};

}  // namespace oak_ffc_camera_imu
