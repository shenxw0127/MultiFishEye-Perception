#include "oak_ffc_camera_imu/image_publisher.h"

#include <chrono>
#include <stdexcept>

#include "sensor_msgs/image_encodings.hpp"

namespace oak_ffc_camera_imu {

SyncedImagePublisher::SyncedImagePublisher(
    rclcpp::Node& node,
    std::shared_ptr<dai::DataOutputQueue> queue,
    std::map<std::string, std::string> topics,
    std::string frame_id_prefix,
    bool compressed,
    bool lazy_publisher)
    : node_(node),
      queue_(std::move(queue)),
      topics_(std::move(topics)),
      frame_id_prefix_(std::move(frame_id_prefix)),
      compressed_(compressed),
      lazy_publisher_(lazy_publisher) {
  rclcpp::QoS qos(rclcpp::KeepLast(3));
  qos.best_effort();

  for (const auto& topic : topics_) {
    if (compressed_) {
      compressed_publishers_[topic.first] =
          node_.create_publisher<sensor_msgs::msg::CompressedImage>(topic.second, qos);
    } else {
      image_publishers_[topic.first] = node_.create_publisher<sensor_msgs::msg::Image>(topic.second, qos);
    }
  }
}

SyncedImagePublisher::~SyncedImagePublisher() {
  stop();
}

void SyncedImagePublisher::start() {
  if (running_.exchange(true)) {
    return;
  }
  thread_ = std::thread(&SyncedImagePublisher::run, this);
}

void SyncedImagePublisher::stop() {
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
}

void SyncedImagePublisher::run() {
  while (rclcpp::ok() && running_) {
    auto group = queue_->tryGet<dai::MessageGroup>();
    if (!group) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    publishGroup(group);
  }
}

void SyncedImagePublisher::publishGroup(const std::shared_ptr<dai::MessageGroup>& group) {
  for (const auto& topic : topics_) {
    const auto& camera_name = topic.first;
    const auto& ros_topic = topic.second;
    if (lazy_publisher_ && node_.count_subscribers(ros_topic) == 0) {
      continue;
    }

    auto frame = group->get<dai::ImgFrame>(camera_name);
    if (!frame) {
      RCLCPP_WARN_THROTTLE(
          node_.get_logger(),
          *node_.get_clock(),
          5000,
          "Synchronized frame group did not contain %s",
          camera_name.c_str());
      continue;
    }

    if (compressed_) {
      compressed_publishers_.at(camera_name)->publish(toCompressedImageMsg(camera_name, frame));
    } else {
      image_publishers_.at(camera_name)->publish(toImageMsg(camera_name, frame));
    }
  }
}

sensor_msgs::msg::Image SyncedImagePublisher::toImageMsg(
    const std::string& camera_name,
    const std::shared_ptr<dai::ImgFrame>& frame) const {
  sensor_msgs::msg::Image msg;
  msg.header.stamp = node_.now();
  msg.header.frame_id = frameId(camera_name);
  msg.width = frame->getWidth();
  msg.height = frame->getHeight();
  msg.is_bigendian = false;

  const auto& data = frame->getData();
  const auto pixel_count = static_cast<std::size_t>(msg.width) * static_cast<std::size_t>(msg.height);
  if (data.size() == pixel_count) {
    msg.encoding = sensor_msgs::image_encodings::MONO8;
    msg.step = msg.width;
  } else if (data.size() == pixel_count * 3 / 2) {
    msg.encoding = sensor_msgs::image_encodings::NV12;
    msg.step = msg.width;
  } else if (data.size() == pixel_count * 3) {
    msg.encoding = sensor_msgs::image_encodings::BGR8;
    msg.step = msg.width * 3;
  } else if (data.size() == pixel_count * 4) {
    msg.encoding = sensor_msgs::image_encodings::BGRA8;
    msg.step = msg.width * 4;
  } else {
    throw std::runtime_error("unsupported OAK image buffer size from DepthAI");
  }
  msg.data = data;
  return msg;
}

sensor_msgs::msg::CompressedImage SyncedImagePublisher::toCompressedImageMsg(
    const std::string& camera_name,
    const std::shared_ptr<dai::ImgFrame>& frame) const {
  sensor_msgs::msg::CompressedImage msg;
  msg.header.stamp = node_.now();
  msg.header.frame_id = frameId(camera_name);
  msg.format = "jpeg";
  msg.data = frame->getData();
  return msg;
}

std::string SyncedImagePublisher::frameId(const std::string& camera_name) const {
  return frame_id_prefix_ + "_" + camera_name + "_optical_frame";
}

}  // namespace oak_ffc_camera_imu
