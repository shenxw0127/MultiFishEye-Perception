#include "recorder/topic_recorder_component.h"

#include <chrono>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>
#include <stdexcept>

#include "rclcpp_components/register_node_macro.hpp"

namespace realsense_ros2 {
namespace {

std::string expandHome(const std::string& path) {
  if (path.empty() || path[0] != '~') {
    return path;
  }
  const char* home = std::getenv("HOME");
  return home ? std::string(home) + path.substr(1) : path;
}

// reliable：与 realsense 驱动的 DEFAULT QoS 匹配（驱动若改成 best_effort，这里要跟着改）。
// best_effort：能匹配任何发布端，但本进程外经网络传大图时可能丢包。
// latched：/tf_static、extrinsics 这类只发一次的话题，transient_local 才能在录制开始时拿到。
rclcpp::QoS parseQos(const std::string& topic, const std::string& kind) {
  rclcpp::QoS qos(rclcpp::KeepLast(10));
  if (kind == "reliable") {
    qos.reliable();
  } else if (kind == "best_effort") {
    qos.best_effort();
  } else if (kind == "latched") {
    qos.reliable().transient_local();
  } else {
    throw std::runtime_error(
        "record_qos for '" + topic + "' must be reliable | best_effort | latched, got '" + kind + "'");
  }
  return qos;
}

}  // namespace

TopicRecorderComponent::TopicRecorderComponent(const rclcpp::NodeOptions& options)
    : rclcpp::Node("topic_recorder", options) {
  bag_root_ = expandHome(declare_parameter<std::string>("bag_root", "~/rosbags"));
  bag_prefix_ = declare_parameter<std::string>("bag_prefix", "d435i");
  storage_id_ = declare_parameter<std::string>("storage_id", "mcap");
  max_duration_s_ = declare_parameter<double>("max_duration_s", 600.0);
  const auto service_name = declare_parameter<std::string>("service_name", "record");
  const auto names = declare_parameter<std::vector<std::string>>("record_topics", std::vector<std::string>{});
  const auto types = declare_parameter<std::vector<std::string>>("record_types", std::vector<std::string>{});
  const auto qos = declare_parameter<std::vector<std::string>>("record_qos", std::vector<std::string>{});

  if (max_duration_s_ <= 0.0) {
    throw std::runtime_error("max_duration_s must be greater than zero");
  }
  if (names.empty()) {
    throw std::runtime_error("record_topics must contain at least one topic");
  }
  if (types.size() != names.size() || qos.size() != names.size()) {
    throw std::runtime_error("record_topics / record_types / record_qos must have the same length");
  }
  for (std::size_t i = 0; i < names.size(); ++i) {
    topics_.push_back({names[i], types[i], parseQos(names[i], qos[i])});
  }

  service_ = create_service<oak_interfaces::srv::Record>(
      service_name,
      std::bind(&TopicRecorderComponent::onRecordRequest, this, std::placeholders::_1, std::placeholders::_2));

  RCLCPP_INFO(
      get_logger(),
      "Topic recorder ready: service=%s bag_root=%s storage=%s topics=%zu",
      service_->get_service_name(),
      bag_root_.c_str(),
      storage_id_.c_str(),
      topics_.size());
}

TopicRecorderComponent::~TopicRecorderComponent() {
  // Ctrl+C 时若仍在录制，照常收尾，保证 bag 的 metadata 被写出。
  if (recording_) {
    stopRecording();
  }
}

void TopicRecorderComponent::onRecordRequest(
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
    const auto bag_path = request->bag_name.empty() ? nextBagPath() : namedBagPath(request->bag_name);
    startRecording(bag_path, request->duration_s);
    response->success = true;
    response->bag_path = bag_path;
    response->message = "recording started";
  } catch (const std::exception& error) {
    RCLCPP_ERROR(get_logger(), "Failed to start recording: %s", error.what());
    subscriptions_.clear();
    {
      std::lock_guard<std::mutex> lock(writer_mutex_);
      writer_.reset();
    }
    response->message = error.what();
  }
}

std::string TopicRecorderComponent::namedBagPath(const std::string& bag_name) const {
  const std::filesystem::path relative(bag_name);
  if (relative.is_absolute()) {
    throw std::runtime_error("bag_name must be relative to bag_root: " + bag_name);
  }
  for (const auto& part : relative) {
    if (part == "..") {
      throw std::runtime_error("bag_name must not contain '..': " + bag_name);
    }
  }
  const auto path = std::filesystem::path(bag_root_) / relative;
  // rosbag2 要求 bag 目录本身不存在，父目录（如同一次联合录制的 session 目录）提前建好。
  if (std::filesystem::exists(path)) {
    throw std::runtime_error("bag already exists: " + path.string());
  }
  std::filesystem::create_directories(path.parent_path());
  return path.string();
}

std::string TopicRecorderComponent::nextBagPath() const {
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

void TopicRecorderComponent::startRecording(const std::string& bag_path, double duration_s) {
  rosbag2_storage::StorageOptions storage_options;
  storage_options.uri = bag_path;
  storage_options.storage_id = storage_id_;
  // 大于 0 时 rosbag2 走缓存 + 独立落盘线程，订阅回调里的 write 只是入缓存，不阻塞 executor。
  storage_options.max_cache_size = 64 * 1024 * 1024;

  auto writer = std::make_unique<rosbag2_cpp::Writer>();
  writer->open(storage_options);
  // 预先建好所有话题并记下 QoS：没收到消息的话题也会出现在 bag 里，一眼能看出缺了什么；
  // latched 话题回放时按 transient_local 重新发布。
  for (const auto& spec : topics_) {
    rosbag2_storage::TopicMetadata metadata;
    metadata.name = spec.name;
    metadata.type = spec.type;
    metadata.serialization_format = "cdr";
    metadata.offered_qos_profiles = {spec.qos};
    writer->create_topic(metadata);
  }
  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    writer_ = std::move(writer);
    counts_.clear();
  }

  for (const auto& spec : topics_) {
    const TopicSpec* spec_ptr = &spec;  // topics_ 构造后不再改动，指针稳定
    subscriptions_.push_back(create_generic_subscription(
        spec.name,
        spec.type,
        spec.qos,
        [this, spec_ptr](std::shared_ptr<const rclcpp::SerializedMessage> message, const rclcpp::MessageInfo& info) {
          onMessage(*spec_ptr, std::move(message), info);
        }));
  }

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

void TopicRecorderComponent::stopRecording() {
  // 先停订阅，不再有新数据进来；再关 writer（析构时把缓存刷盘并写 metadata）。
  subscriptions_.clear();
  if (stop_timer_) {
    stop_timer_->cancel();
    stop_timer_.reset();
  }

  std::map<std::string, std::uint64_t> counts;
  {
    std::lock_guard<std::mutex> lock(writer_mutex_);
    try {
      writer_.reset();
    } catch (const std::exception& error) {
      RCLCPP_ERROR(get_logger(), "Error while closing bag %s: %s", bag_path_.c_str(), error.what());
    }
    counts = counts_;
  }
  recording_ = false;

  std::ostringstream summary;
  for (const auto& spec : topics_) {
    const auto it = counts.find(spec.name);
    const auto n = it == counts.end() ? 0 : it->second;
    summary << "\n  " << spec.name << ": " << n;
    if (n == 0) {
      summary << "  <-- 没收到（相机没接？话题名/类型/QoS 不匹配？）";
    }
  }
  RCLCPP_INFO(
      get_logger(), "\033[0;32mRecording finished: \033[0m%s%s", bag_path_.c_str(), summary.str().c_str());
}

void TopicRecorderComponent::onMessage(
    const TopicSpec& spec,
    std::shared_ptr<const rclcpp::SerializedMessage> message,
    const rclcpp::MessageInfo& info) {
  const auto recv_ns = get_clock()->now().nanoseconds();
  const auto source_ns = info.get_rmw_message_info().source_timestamp;
  const auto send_ns = source_ns > 0 ? source_ns : recv_ns;

  std::lock_guard<std::mutex> lock(writer_mutex_);
  if (!writer_) {
    return;
  }
  try {
    writer_->write(std::move(message), spec.name, spec.type, recv_ns, send_ns);
    ++counts_[spec.name];
  } catch (const std::exception& error) {
    RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000, "Failed to write %s: %s", spec.name.c_str(), error.what());
  }
}

}  // namespace realsense_ros2

RCLCPP_COMPONENTS_REGISTER_NODE(realsense_ros2::TopicRecorderComponent)
