#include "recorder/bag_recorder_component.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include "opencv2/imgcodecs.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"

namespace oak_ffc_camera_imu {
namespace {

std::string expandHome(const std::string& path) {
  if (path.empty() || path[0] != '~') {
    return path;
  }
  const char* home = std::getenv("HOME");
  return home ? std::string(home) + path.substr(1) : path;
}

bool hasValidStorage(const sensor_msgs::msg::Image& msg) {
  return msg.height > 0 && msg.width > 0 && msg.step > 0 &&
         msg.data.size() >= static_cast<std::size_t>(msg.step) * msg.height;
}

}  // namespace

BagRecorderComponent::BagRecorderComponent(const rclcpp::NodeOptions& options)
    : rclcpp::Node("oak_bag_recorder", options) {
  bag_root_ = expandHome(declare_parameter<std::string>("bag_root", "~/rosbags"));
  bag_prefix_ = declare_parameter<std::string>("bag_prefix", "fisheye4");
  storage_id_ = declare_parameter<std::string>("storage_id", "mcap");
  jpeg_quality_ = static_cast<int>(declare_parameter<int64_t>("jpeg_quality", 90));
  const auto jpeg_threads = declare_parameter<int64_t>("jpeg_threads", 2);
  const auto max_queue = declare_parameter<int64_t>("max_queue", 60);
  max_duration_s_ = declare_parameter<double>("max_duration_s", 600.0);
  cam_board_sockets_ = declare_parameter<std::vector<std::string>>(
      "cam_board_sockets", std::vector<std::string>{"CAM_A", "CAM_B", "CAM_C", "CAM_D"});
  imu_topic_ = declare_parameter<std::string>("imu_topic", "/imu");

  if (jpeg_quality_ < 1 || jpeg_quality_ > 100) {
    throw std::runtime_error("jpeg_quality must be within 1..100");
  }
  if (jpeg_threads < 1 || jpeg_threads > 8) {
    throw std::runtime_error("jpeg_threads must be within 1..8");
  }
  if (max_queue < 1) {
    throw std::runtime_error("max_queue must be greater than zero");
  }
  if (max_duration_s_ <= 0.0) {
    throw std::runtime_error("max_duration_s must be greater than zero");
  }
  if (cam_board_sockets_.empty()) {
    throw std::runtime_error("cam_board_sockets must contain at least one camera socket");
  }
  max_queue_ = static_cast<std::size_t>(max_queue);

  for (int64_t i = 0; i < jpeg_threads; ++i) {
    workers_.emplace_back(&BagRecorderComponent::encodeWorker, this);
  }

  service_ = create_service<oak_interfaces::srv::Record>(
      "/oak/record",
      std::bind(&BagRecorderComponent::onRecordRequest, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
      get_logger(),
      "Bag recorder ready: service=/oak/record bag_root=%s storage=%s jpeg_quality=%d jpeg_threads=%ld",
      bag_root_.c_str(),
      storage_id_.c_str(),
      jpeg_quality_,
      static_cast<long>(jpeg_threads));
}

BagRecorderComponent::~BagRecorderComponent() {
  // Ctrl+C 时若仍在录制，照常收尾，保证 bag 的 metadata 被写出。
  if (recording_) {
    stopRecording();
  }
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    workers_running_ = false;
  }
  queue_cv_.notify_all();
  for (auto& worker : workers_) {
    if (worker.joinable()) {
      worker.join();
    }
  }
}

void BagRecorderComponent::onRecordRequest(
    const std::shared_ptr<oak_interfaces::srv::Record::Request> request,
    std::shared_ptr<oak_interfaces::srv::Record::Response> response) {
  response->success = false;
  if (recording_) {
    response->message = "already recording to " + bag_path_;
    response->bag_path = bag_path_;
    return;
  }
  if (!(request->duration_s > 0.0) || request->duration_s > max_duration_s_) {
    std::ostringstream message;
    message << "duration_s must be within (0, " << max_duration_s_ << "]";
    response->message = message.str();
    return;
  }

  try {
    const auto bag_path = nextBagPath();
    startRecording(bag_path, request->duration_s);
    response->success = true;
    response->bag_path = bag_path;
    response->message = "recording started";
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "Failed to start recording: %s", error.what());
    response->message = error.what();
  }
}

std::string BagRecorderComponent::nextBagPath() const {
  std::filesystem::create_directories(bag_root_);

  const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  std::tm local_time{};
  localtime_r(&now, &local_time);
  std::ostringstream name;
  name << bag_prefix_ << "_" << std::put_time(&local_time, "%Y%m%d_%H%M%S");

  // rosbag2 要求目录不存在；同一秒内重复触发时追加 _N。
  const auto base = (std::filesystem::path(bag_root_) / name.str()).string();
  auto candidate = base;
  for (int n = 1; std::filesystem::exists(candidate); ++n) {
    candidate = base + "_" + std::to_string(n);
  }
  return candidate;
}

void BagRecorderComponent::startRecording(const std::string& bag_path, double duration_s) {
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path;
  storage_options.storage_id = storage_id_;
  // 及时落盘：异常断电时丢的数据更少，收尾也更快。
  storage_options.max_cache_size = 10 * 1024 * 1024;

  auto writer = std::make_unique<rosbag2_cpp::Writer>();
  writer->open(storage_options);
  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    writer_ = std::move(writer);
  }

  images_written_ = 0;
  imu_written_ = 0;
  images_dropped_ = 0;

  // 同进程里有不支持 intra-process 的订阅者（如 foxglove_bridge 的 generic subscription）时，
  // 驱动会同时走 intra-process 和 DDS 发布；rclcpp 按 gid 去重对大图像消息失效，实测每帧录两份。
  // 让 DDS 层丢掉本进程的样本：intra-process 投递不经过 rmw 不受影响，其他进程的发布照常接收。
  rclcpp::SubscriptionOptions sub_options;
  sub_options.ignore_local_publications = true;

  // QoS 与驱动发布端一致（best_effort），否则 intra-process 匹配不上。
  rclcpp::QoS image_qos(rclcpp::KeepLast(5));
  image_qos.best_effort();
  for (const auto& socket : cam_board_sockets_) {
    const auto input_topic = "/" + socket + "/image";
    const auto output_topic = input_topic + "/compressed";
    subscriptions_.push_back(create_subscription<sensor_msgs::msg::Image>(
        input_topic,
        image_qos,
        [this, output_topic](sensor_msgs::msg::Image::ConstSharedPtr image) {
          onImage(output_topic, std::move(image));
        },
        sub_options));
  }

  rclcpp::QoS imu_qos(rclcpp::KeepLast(200));
  imu_qos.best_effort();
  subscriptions_.push_back(create_subscription<sensor_msgs::msg::Imu>(
      imu_topic_,
      imu_qos,
      [this](sensor_msgs::msg::Imu::ConstSharedPtr imu) { onImu(std::move(imu)); },
      sub_options));

  stop_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(duration_s)),
      [this]() {
        stop_timer_->cancel();
        stopRecording();
      });

  recording_ = true;
  bag_path_ = bag_path;
  RCLCPP_INFO(get_logger(), "Recording %.1fs to %s", duration_s, bag_path.c_str());
}

void BagRecorderComponent::stopRecording() {
  // 先停订阅，不再有新数据进来；再等编码队列排空；最后关 writer 写 metadata。
  subscriptions_.clear();
  if (stop_timer_) {
    stop_timer_->cancel();
    stop_timer_.reset();
  }
  {
    std::unique_lock<std::mutex> lock(queue_mutex_);
    idle_cv_.wait(lock, [this] { return queue_.empty() && in_flight_ == 0; });
  }
  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    try {
      writer_.reset();
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "Error while closing bag %s: %s", bag_path_.c_str(), error.what());
    }
  }

  recording_ = false;
  RCLCPP_INFO(
      get_logger(),
      "\033[0;32mRecording finished: \033[0m%s (images=%lu imu=%lu dropped=%lu)",
      bag_path_.c_str(),
      static_cast<unsigned long>(images_written_.load()),
      static_cast<unsigned long>(imu_written_.load()),
      static_cast<unsigned long>(images_dropped_.load()));
}

void BagRecorderComponent::onImage(const std::string& output_topic, sensor_msgs::msg::Image::ConstSharedPtr image) {
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    if (queue_.size() >= max_queue_) {
      ++images_dropped_;
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "JPEG encoder falling behind, dropped %lu frames so far",
          static_cast<unsigned long>(images_dropped_.load()));
      return;
    }
    queue_.push_back({output_topic, std::move(image)});
  }
  queue_cv_.notify_one();
}

void BagRecorderComponent::onImu(sensor_msgs::msg::Imu::ConstSharedPtr imu) {
  std::lock_guard<std::mutex> lock(writer_mutex_);
  if (!writer_) {
    return;
  }
  try {
    writer_->write(*imu, "/imu", rclcpp::Time(imu->header.stamp));
    ++imu_written_;
  } catch (const std::exception& error) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to write IMU: %s", error.what());
  }
}

void BagRecorderComponent::encodeWorker() {
  const std::vector<int> encode_params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};

  while (true) {
    EncodeJob job;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] { return !queue_.empty() || !workers_running_; });
      if (queue_.empty()) {
        return;  // workers_running_ == false
      }
      job = std::move(queue_.front());
      queue_.pop_front();
      ++in_flight_;
    }

    const auto& image = *job.image;
    int cv_type = -1;
    if (image.encoding == sensor_msgs::image_encodings::BGR8) {
      cv_type = CV_8UC3;
    } else if (image.encoding == sensor_msgs::image_encodings::MONO8) {
      cv_type = CV_8UC1;
    }

    sensor_msgs::msg::CompressedImage compressed;
    bool ok = false;
    if (cv_type < 0 || !hasValidStorage(image)) {
      RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000, "Skipping image with unsupported encoding '%s'", image.encoding.c_str());
    } else {
      try {
        // 只读包装原始 buffer，不拷贝。
        const cv::Mat mat(
            static_cast<int>(image.height),
            static_cast<int>(image.width),
            cv_type,
            const_cast<uint8_t*>(image.data.data()),
            image.step);
        ok = cv::imencode(".jpg", mat, compressed.data, encode_params);
      } catch (const cv::Exception& error) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "JPEG encode failed: %s", error.what());
      }
    }

    if (ok) {
      compressed.header = image.header;
      compressed.format = "jpeg";
      std::lock_guard<std::mutex> lock(writer_mutex_);
      if (writer_) {
        try {
          writer_->write(compressed, job.topic, rclcpp::Time(image.header.stamp));
          ++images_written_;
        } catch (const std::exception& error) {
          RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "Failed to write image: %s", error.what());
        }
      }
    }

    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      --in_flight_;
    }
    idle_cv_.notify_all();
  }
}

}  // namespace oak_ffc_camera_imu

RCLCPP_COMPONENTS_REGISTER_NODE(oak_ffc_camera_imu::BagRecorderComponent)
