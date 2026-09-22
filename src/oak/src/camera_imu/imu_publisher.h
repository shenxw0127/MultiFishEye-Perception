#pragma once

#include <atomic>
#include <memory>
#include <string>
#include <thread>

#include "depthai/depthai.hpp"
#include "camera_imu/dai_time.h"
#include "camera_imu/imu_correction.h"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace oak_ffc_camera_imu {

class ImuPublisher {
 public:
  ImuPublisher(
      rclcpp::Node& node,
      std::string topic,
      std::string frame_id,
      ImuCorrection correction,
      std::shared_ptr<DaiTimeConverter> time_converter);
  ~ImuPublisher();

  ImuPublisher(const ImuPublisher&) = delete;
  ImuPublisher& operator=(const ImuPublisher&) = delete;

  // 同 SyncedImagePublisher：attach 挂新设备队列，detach 停线程，healthy 反映断连。
  void attach(std::shared_ptr<dai::DataOutputQueue> queue);
  void detach();
  bool healthy() const { return healthy_; }

 private:
  void run();
  sensor_msgs::msg::Imu toRosMsg(const dai::IMUPacket& packet) const;

  rclcpp::Node& node_;
  std::shared_ptr<dai::DataOutputQueue> queue_;
  std::string frame_id_;
  ImuCorrection correction_;
  std::shared_ptr<DaiTimeConverter> time_converter_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr publisher_;
  std::atomic_bool running_{false};
  std::atomic_bool healthy_{false};
  std::thread thread_;
};

}  // namespace oak_ffc_camera_imu
