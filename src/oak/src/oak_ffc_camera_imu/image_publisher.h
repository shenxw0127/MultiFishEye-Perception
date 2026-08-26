#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <string>
#include <thread>

#include "depthai/depthai.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace oak_ffc_camera_imu {

class SyncedImagePublisher {
 public:
  SyncedImagePublisher(
      rclcpp::Node& node,
      std::shared_ptr<dai::DataOutputQueue> queue,
      std::map<std::string, std::string> topics,
      std::string frame_id_prefix,
      bool compressed,
      bool lazy_publisher);
  ~SyncedImagePublisher();

  SyncedImagePublisher(const SyncedImagePublisher&) = delete;
  SyncedImagePublisher& operator=(const SyncedImagePublisher&) = delete;

  void start();
  void stop();

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
  std::atomic_bool running_{false};
  std::thread thread_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_publishers_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> compressed_publishers_;
};

}  // namespace oak_ffc_camera_imu
