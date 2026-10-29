#include "camera_imu/image_publisher.h"

#include <chrono>
#include <stdexcept>

#include "camera_imu/isp_meta.h"
#include "opencv2/imgcodecs.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace oak_ffc_camera_imu {
namespace {

constexpr const char* kEncodingNv12 = "nv12";
// 与 imu 一样是相对话题名，组件挂在根命名空间时就是 /isp_meta。
constexpr const char* kIspMetaTopic = "isp_meta";
// 连续这么久收不到同步组就判为停流（含 attach 后等首组：实测首组 < 1s）。
constexpr auto kStallTimeout = std::chrono::milliseconds(3000);

}  // namespace

SyncedImagePublisher::SyncedImagePublisher(
    rclcpp::Node& node,
    std::map<std::string, std::string> topics,
    std::string frame_id_prefix,
    bool compressed,
    bool lazy_publisher,
    JpegDecode jpeg_decode,
    int image_width,
    int image_height,
    std::shared_ptr<DaiTimeConverter> time_converter)
    : node_(node),
      topics_(std::move(topics)),
      frame_id_prefix_(std::move(frame_id_prefix)),
      compressed_(compressed),
      lazy_publisher_(lazy_publisher),
      jpeg_decode_(jpeg_decode),
      image_width_(image_width),
      image_height_(image_height),
      time_converter_(std::move(time_converter)) {
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
  // 每组一条、几十字节，可靠传输不会成为负担；ros2 topic echo / foxglove 的默认订阅都能直接匹配。
  isp_meta_publisher_ =
      node_.create_publisher<oak_interfaces::msg::IspMetaArray>(kIspMetaTopic, rclcpp::QoS(rclcpp::KeepLast(10)));
}

SyncedImagePublisher::~SyncedImagePublisher() {
  detach();
}

void SyncedImagePublisher::attach(
    std::shared_ptr<dai::DataOutputQueue> queue,
    std::vector<std::string> active_sockets) {
  detach();
  for (const auto& socket : active_sockets) {
    if (topics_.find(socket) == topics_.end()) {
      throw std::logic_error("camera socket has no ROS publisher: " + socket);
    }
  }
  queue_ = std::move(queue);
  {
    std::lock_guard<std::mutex> lock(frame_counts_mutex_);
    for (const auto& socket : active_sockets) {
      frame_counts_[socket] = 0;
    }
  }
  // 读线程启动前写好，之后只读，无需加锁。
  active_sockets_ = std::move(active_sockets);
  healthy_ = true;
  running_ = true;
  thread_ = std::thread(&SyncedImagePublisher::run, this);
}

void SyncedImagePublisher::detach() {
  running_ = false;
  if (thread_.joinable()) {
    thread_.join();
  }
  queue_.reset();
  healthy_ = false;
  std::lock_guard<std::mutex> lock(frame_counts_mutex_);
  frame_counts_.clear();
}

FrameCounts SyncedImagePublisher::frameCounts() const {
  std::lock_guard<std::mutex> lock(frame_counts_mutex_);
  return frame_counts_;
}

void SyncedImagePublisher::run() {
  // 设备断连时 DataOutputQueue 会抛 std::runtime_error。这里吞掉并标记 unhealthy，
  // 交给 supervisor 重连；绝不能让异常逃出线程，否则 std::terminate 会带走整个 container。
  // 某一路相机中途停止出帧（排线松动、sensor 掉电）时 Sync 等不齐一组，队列永远为空，
  // 设备却仍在线，所以靠超时判停流，交给 supervisor 重连并重新探测。
  try {
    auto last_group = std::chrono::steady_clock::now();
    while (rclcpp::ok() && running_) {
      auto group = queue_->tryGet<dai::MessageGroup>();
      if (!group) {
        if (std::chrono::steady_clock::now() - last_group > kStallTimeout) {
          RCLCPP_WARN(
              node_.get_logger(), "No synchronized frame group for %lld ms, a camera stopped streaming",
              static_cast<long long>(kStallTimeout.count()));
          healthy_ = false;
          return;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
      last_group = std::chrono::steady_clock::now();
      publishGroup(group);
    }
  } catch (const std::exception& error) {
    RCLCPP_WARN(node_.get_logger(), "Image stream stopped: %s", error.what());
    healthy_ = false;
  }
}

void SyncedImagePublisher::publishGroup(const std::shared_ptr<dai::MessageGroup>& group) {
  // 先把本组所有帧取出来：图像按话题各自发布（可能因无订阅者而跳过），ISP 元数据则要用到全部帧。
  std::map<std::string, std::shared_ptr<dai::ImgFrame>> frames;
  for (const auto& camera_name : active_sockets_) {
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
    frames[camera_name] = std::move(frame);
  }
  {
    std::lock_guard<std::mutex> lock(frame_counts_mutex_);
    for (const auto& entry : frames) {
      ++frame_counts_[entry.first];
    }
  }

  for (const auto& entry : frames) {
    const auto& camera_name = entry.first;
    const auto& frame = entry.second;
    const auto& ros_topic = topics_.at(camera_name);
    if (lazy_publisher_ && node_.count_subscribers(ros_topic) == 0) {
      continue;
    }

    if (compressed_) {
      compressed_publishers_.at(camera_name)->publish(toCompressedImageMsg(camera_name, frame));
    } else if (jpeg_decode_ != JpegDecode::None) {
      // 单帧解码失败只丢这一帧，不能抛到 run() 里，否则会触发整机重连、四路一起断几秒。
      auto msg = decodeJpegToImageMsg(camera_name, frame);
      if (msg.data.empty()) {
        RCLCPP_WARN_THROTTLE(
            node_.get_logger(), *node_.get_clock(), 5000, "Dropped undecodable MJPEG frame from %s",
            camera_name.c_str());
        continue;
      }
      image_publishers_.at(camera_name)->publish(std::move(msg));
    } else {
      image_publishers_.at(camera_name)->publish(toImageMsg(camera_name, frame));
    }
  }

  publishIspMeta(frames);
}

void SyncedImagePublisher::publishIspMeta(const std::map<std::string, std::shared_ptr<dai::ImgFrame>>& frames) {
  if (frames.empty() || (lazy_publisher_ && isp_meta_publisher_->get_subscription_count() == 0)) {
    return;
  }

  oak_interfaces::msg::IspMetaArray msg;
  msg.header.frame_id = frame_id_prefix_;
  msg.cameras.reserve(frames.size());
  bool any_meta = false;
  // frames 是 std::map，按插座名字典序（CAM_A..D）。
  for (const auto& entry : frames) {
    const auto stamp = time_converter_->toRosTime(entry.second->getTimestamp(), node_.get_clock());
    if (msg.cameras.empty()) {
      msg.header.stamp = stamp;
    }
    msg.cameras.push_back(toIspMeta(entry.first, *entry.second, stamp));
    any_meta = any_meta || hasIspMeta(msg.cameras.back());
  }

  if (!any_meta && !isp_meta_missing_warned_) {
    isp_meta_missing_warned_ = true;
    RCLCPP_WARN(
        node_.get_logger(),
        "ISP metadata is all zero on this stream (exposure/ISO/WB not filled by firmware on the %s path); "
        "isp_meta will carry zeros.",
        jpeg_decode_ != JpegDecode::None || compressed_ ? "MJPEG encoder" : "raw");
  }
  isp_meta_publisher_->publish(std::move(msg));
}

sensor_msgs::msg::Image SyncedImagePublisher::toImageMsg(
    const std::string& camera_name,
    const std::shared_ptr<dai::ImgFrame>& frame) const {
  sensor_msgs::msg::Image msg;
  // DepthAI hardware timestamp, shared time base with the IMU stream.
  msg.header.stamp = time_converter_->toRosTime(frame->getTimestamp(), node_.get_clock());
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
    msg.encoding = kEncodingNv12;
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

sensor_msgs::msg::Image SyncedImagePublisher::decodeJpegToImageMsg(
    const std::string& camera_name,
    const std::shared_ptr<dai::ImgFrame>& frame) const {
  // 设备端 Hantro 编码器固定出 3 分量 YUV420 JPEG；灰度 sensor 的色度恒为 128，
  // IMREAD_GRAYSCALE 只解 Y 分量，得到的就是 sensor 亮度，不存在彩色转灰度。
  const bool mono = jpeg_decode_ == JpegDecode::Mono8;
  // 解码失败或尺寸不对时返回 data 为空的消息，由调用方丢帧。
  const auto& data = frame->getData();
  const cv::Mat encoded(1, static_cast<int>(data.size()), CV_8UC1, const_cast<std::uint8_t*>(data.data()));
  const int width = image_width_;
  const int height = image_height_;
  const int channels = mono ? 1 : 3;

  sensor_msgs::msg::Image msg;
  msg.header.stamp = time_converter_->toRosTime(frame->getTimestamp(), node_.get_clock());
  msg.header.frame_id = frameId(camera_name);
  msg.width = static_cast<std::uint32_t>(width);
  msg.height = static_cast<std::uint32_t>(height);
  msg.is_bigendian = false;
  msg.encoding = mono ? sensor_msgs::image_encodings::MONO8 : sensor_msgs::image_encodings::BGR8;
  msg.step = static_cast<std::uint32_t>(width * channels);
  msg.data.resize(static_cast<std::size_t>(msg.step) * static_cast<std::size_t>(height));

  // 直接解码进消息缓冲区，省掉一次整帧拷贝；imdecode 发现尺寸不符会重新分配，借此检测。
  cv::Mat dst(height, width, mono ? CV_8UC1 : CV_8UC3, msg.data.data());
  const auto* const buffer = dst.data;
  try {
    cv::imdecode(encoded, mono ? cv::IMREAD_GRAYSCALE : cv::IMREAD_COLOR, &dst);
  } catch (const cv::Exception&) {
    dst.release();
  }
  if (dst.empty() || dst.data != buffer) {
    msg.data.clear();
  }
  return msg;
}

sensor_msgs::msg::CompressedImage SyncedImagePublisher::toCompressedImageMsg(
    const std::string& camera_name,
    const std::shared_ptr<dai::ImgFrame>& frame) const {
  sensor_msgs::msg::CompressedImage msg;
  // DepthAI hardware timestamp, shared time base with the IMU stream.
  msg.header.stamp = time_converter_->toRosTime(frame->getTimestamp(), node_.get_clock());
  msg.header.frame_id = frameId(camera_name);
  msg.format = "jpeg";
  msg.data = frame->getData();
  return msg;
}

std::string SyncedImagePublisher::frameId(const std::string& camera_name) const {
  return frame_id_prefix_ + "_" + camera_name + "_optical_frame";
}

}  // namespace oak_ffc_camera_imu
