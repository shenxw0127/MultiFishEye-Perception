#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "depthai/depthai.hpp"
#include "camera_imu/dai_time.h"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace oak_ffc_camera_imu {

class SyncedImagePublisher {
 public:
  SyncedImagePublisher(
      rclcpp::Node& node,
      std::map<std::string, std::string> topics,
      std::string frame_id_prefix,
      bool compressed,
      bool lazy_publisher,
      std::shared_ptr<DaiTimeConverter> time_converter);
  ~SyncedImagePublisher();

  SyncedImagePublisher(const SyncedImagePublisher&) = delete;
  SyncedImagePublisher& operator=(const SyncedImagePublisher&) = delete;

  // 挂到一个新连上的设备队列并起读线程；ROS publisher 在构造时建好，重连不重建。
  void attach(std::shared_ptr<dai::DataOutputQueue> queue);
  // 停读线程并释放队列，可重复调用。
  void detach();
  // 读线程因设备异常（断连）退出后变为 false，由 supervisor 轮询。
  bool healthy() const { return healthy_; }

 private:
  void run();
  void publishGroup(const std::shared_ptr<dai::MessageGroup>& group);
  sensor_msgs::msg::Image toImageMsg(const std::string& camera_name, const std::shared_ptr<dai::ImgFrame>& frame) const;
  sensor_msgs::msg::CompressedImage toCompressedImageMsg(
      const std::string& camera_name,
      const std::shared_ptr<dai::ImgFrame>& frame) const;
  std::string frameId(const std::string& camera_name) const;

  rclcpp::Node& node_;
  std::shared_ptr<dai::DataOutputQueue> queue_;
  std::map<std::string, std::string> topics_;
  std::string frame_id_prefix_;
  bool compressed_;
  bool lazy_publisher_;
  std::shared_ptr<DaiTimeConverter> time_converter_;
  std::atomic_bool running_{false};
  std::atomic_bool healthy_{false};
  std::thread thread_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_publishers_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> compressed_publishers_;
};

}  // namespace oak_ffc_camera_imu
