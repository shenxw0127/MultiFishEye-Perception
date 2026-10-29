#include "camera_imu/imu_publisher.h"

#include <chrono>

namespace oak_ffc_camera_imu {
namespace {

// 正常连接后约 220ms 出第一包，3s 足够区分"慢"和"卡死"。
constexpr auto kImuStallTimeout = std::chrono::seconds(3);

}  // namespace

ImuPublisher::ImuPublisher(
    rclcpp::Node& node,
    std::string topic,
    std::string frame_id,
    ImuCorrection correction,
    std::shared_ptr<DaiTimeConverter> time_converter)
    : node_(node),
      frame_id_(std::move(frame_id)),
      correction_(std::move(correction)),
      time_converter_(std::move(time_converter)) {
  rclcpp::QoS qos(rclcpp::KeepLast(50));
  qos.best_effort();
  publisher_ = node_.create_publisher<sensor_msgs::msg::Imu>(std::move(topic), qos);
}

ImuPublisher::~ImuPublisher() {
  detach();
}

void ImuPublisher::attach(std::shared_ptr<dai::DataOutputQueue> queue) {
  detach();
  queue_ = std::move(queue);
  packet_count_ = 0;
  attached_ = true;
  healthy_ = true;
  running_ = true;
  thread_ = std::thread(&ImuPublisher::run, this);
}

void ImuPublisher::detach() {
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
  queue_.reset();
  healthy_ = false;
  attached_ = false;
}

std::optional<std::uint64_t> ImuPublisher::packetCount() const {
  if (!attached_) {
    return std::nullopt;
  }
  return packet_count_.load();
}

void ImuPublisher::run() {
  // 断连异常在线程内消化，见 SyncedImagePublisher::run。
  //
  // 看门狗：实测连接后 IMU 偶发完全不出数（图像正常）；设备刚被上一个进程关掉就重新连接时
  // 几乎必现，重连一次后约 220ms 出第一包。旧版录制 imu=0 就是这个原因。
  // attach 后或运行中超过 kImuStallTimeout 收不到包就标记 unhealthy，交给 supervisor
  // 整机重连，和拔插走同一条恢复路径。
  auto last_data = std::chrono::steady_clock::now();
  bool first_packet = true;
  try {
    while (rclcpp::ok() && running_) {
      auto data = queue_->tryGet<dai::IMUData>();
      if (!data) {
        if (std::chrono::steady_clock::now() - last_data > kImuStallTimeout) {
          RCLCPP_WARN(
              node_.get_logger(),
              "No IMU data for %lds, forcing OAK reconnect",
              static_cast<long>(std::chrono::duration_cast<std::chrono::seconds>(kImuStallTimeout).count()));
          healthy_ = false;
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      if (first_packet) {
        first_packet = false;
        RCLCPP_INFO(
            node_.get_logger(),
            "First IMU packet %ld ms after attach",
            static_cast<long>(
                std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - last_data)
                    .count()));
      }
      last_data = std::chrono::steady_clock::now();

      for (const auto& packet : data->packets) {
        publisher_->publish(toRosMsg(packet));
      }
      packet_count_ += data->packets.size();
    }
  } catch (const std::exception& error) {
    RCLCPP_WARN(node_.get_logger(), "IMU stream stopped: %s", error.what());
    healthy_ = false;
  }
}

sensor_msgs::msg::Imu ImuPublisher::toRosMsg(const dai::IMUPacket& packet) const {
  sensor_msgs::msg::Imu msg;
  // Use the DepthAI hardware timestamp (device clock) rather than the host
  // receive time. IMU packets arrive from the device in batches, so stamping
  // with node_.now() collapses many samples onto near-identical times and
  // breaks the monotonic dt that OpenVINS' propagator requires.
  msg.header.stamp = time_converter_->toRosTime(packet.acceleroMeter.getTimestamp(), node_.get_clock());
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

  correction_.apply(msg);
  return msg;
}

}  // namespace oak_ffc_camera_imu
