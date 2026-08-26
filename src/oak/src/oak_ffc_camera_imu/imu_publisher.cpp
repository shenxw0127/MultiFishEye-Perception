#include "oak_ffc_camera_imu/imu_publisher.h"

#include <chrono>

namespace oak_ffc_camera_imu {

ImuPublisher::ImuPublisher(
    rclcpp::Node& node,
    std::shared_ptr<dai::DataOutputQueue> queue,
    std::string topic,
    std::string frame_id)
    : node_(node), queue_(std::move(queue)), frame_id_(std::move(frame_id)) {
  rclcpp::QoS qos(rclcpp::KeepLast(50));
  qos.best_effort();
  publisher_ = node_.create_publisher<sensor_msgs::msg::Imu>(std::move(topic), qos);
}

ImuPublisher::~ImuPublisher() {
  stop();
}

void ImuPublisher::start() {
  if (running_.exchange(true)) {
    return;
  }
  thread_ = std::thread(&ImuPublisher::run, this);
}

void ImuPublisher::stop() {
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
}

void ImuPublisher::run() {
  while (rclcpp::ok() && running_) {
    auto data = queue_->tryGet<dai::IMUData>();
    if (!data) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    for (const auto& packet : data->packets) {
      publisher_->publish(toRosMsg(packet));
    }
  }
}

sensor_msgs::msg::Imu ImuPublisher::toRosMsg(const dai::IMUPacket& packet) const {
  sensor_msgs::msg::Imu msg;
  msg.header.stamp = node_.now();
  msg.header.frame_id = frame_id_;

  msg.orientation.w = packet.rotationVector.real;
  msg.orientation.x = packet.rotationVector.i;
  msg.orientation.y = packet.rotationVector.j;
  msg.orientation.z = packet.rotationVector.k;

  msg.angular_velocity.x = packet.gyroscope.x;
  msg.angular_velocity.y = packet.gyroscope.y;
  msg.angular_velocity.z = packet.gyroscope.z;

  msg.linear_acceleration.x = packet.acceleroMeter.x;
  msg.linear_acceleration.y = packet.acceleroMeter.y;
  msg.linear_acceleration.z = packet.acceleroMeter.z;

  return msg;
}

}  // namespace oak_ffc_camera_imu
