#pragma once

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "camera_imu/dai_time.h"
#include "camera_imu/fps_display.h"
#include "oak_interfaces/msg/isp_meta_array.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace oak_ffc_camera_imu {

// 设备送来的是 MJPEG、但 ROS 端要发裸图时，主机按此方式解码。
enum class JpegDecode {
  None,   // 设备送来就是裸图（或 ROS 端直接发 JPEG）
  Mono8,  // 只解亮度，出 mono8
  Bgr8,
};

class SyncedImagePublisher {
 public:
  SyncedImagePublisher(
      rclcpp::Node& node,
      std::map<std::string, std::string> topics,
      std::string frame_id_prefix,
      bool compressed,
      bool lazy_publisher,
      JpegDecode jpeg_decode,
      int image_width,
      int image_height,
      std::shared_ptr<DaiTimeConverter> time_converter);
  ~SyncedImagePublisher();

  SyncedImagePublisher(const SyncedImagePublisher&) = delete;
  SyncedImagePublisher& operator=(const SyncedImagePublisher&) = delete;

  // 挂到一个新连上的设备队列并起读线程；ROS publisher 在构造时建好，重连不重建。
  // active_sockets：本次 pipeline 实际在跑的插座（topics 的子集，ALL 模式下按检测结果而定）。
  void attach(std::shared_ptr<dai::DataOutputQueue> queue, std::vector<std::string> active_sockets);
  // 停读线程并释放队列，可重复调用。
  void detach();
  // 读线程因设备异常（断连）退出后变为 false，由 supervisor 轮询。
  bool healthy() const { return healthy_; }
  // 本次 attach 以来各在跑插座从设备收到的帧数，不受 lazy_publisher 影响，供终端帧率显示。
  // 未 attach 时为空。可在任意线程调用。
  FrameCounts frameCounts() const;

 private:
  void run();
  void publishGroup(const std::shared_ptr<dai::MessageGroup>& group);
  // 把本组每路帧的 ISP 元数据（曝光 / ISO / 白平衡色温 / 对焦）打包成一条 IspMetaArray 发到 isp_meta。
  void publishIspMeta(const std::map<std::string, std::shared_ptr<dai::ImgFrame>>& frames);
  sensor_msgs::msg::Image toImageMsg(const std::string& camera_name, const std::shared_ptr<dai::ImgFrame>& frame) const;
  sensor_msgs::msg::Image decodeJpegToImageMsg(
      const std::string& camera_name,
      const std::shared_ptr<dai::ImgFrame>& frame) const;
  sensor_msgs::msg::CompressedImage toCompressedImageMsg(
      const std::string& camera_name,
      const std::shared_ptr<dai::ImgFrame>& frame) const;
  std::string frameId(const std::string& camera_name) const;

  rclcpp::Node& node_;
  std::shared_ptr<dai::DataOutputQueue> queue_;
  std::map<std::string, std::string> topics_;
  std::vector<std::string> active_sockets_;
  std::string frame_id_prefix_;
  bool compressed_;
  bool lazy_publisher_;
  JpegDecode jpeg_decode_;
  // MJPEG 码流帧的 ImgFrame 宽高无意义，解码尺寸以配置分辨率为准。
  int image_width_;
  int image_height_;
  std::shared_ptr<DaiTimeConverter> time_converter_;
  std::atomic_bool running_{false};
  std::atomic_bool healthy_{false};
  std::thread thread_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr> image_publishers_;
  std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr> compressed_publishers_;
  rclcpp::Publisher<oak_interfaces::msg::IspMetaArray>::SharedPtr isp_meta_publisher_;
  // 固件没在这条通路上填元数据（全 0）时只告警一次。
  bool isp_meta_missing_warned_ = false;
  mutable std::mutex frame_counts_mutex_;
  FrameCounts frame_counts_;
};

}  // namespace oak_ffc_camera_imu
