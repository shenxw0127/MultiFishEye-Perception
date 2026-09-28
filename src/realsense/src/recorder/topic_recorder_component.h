#pragma once

#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "oak_interfaces/srv/record.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"

namespace realsense_ros2 {

// 按需定时录制：收到 Record 服务请求后录 duration_s 秒 rosbag，到时自动收尾。
//
// 录哪些话题完全由参数给出（record_topics / record_types / record_qos 三个等长数组，
// 由 launch/realsense_params.py 的 RECORD_TOPICS 生成），用 generic subscription 直接拿
// 序列化数据写盘，不需要为每种消息类型编译代码，加话题只改 Python 列表。
//
// 只在录制期间订阅：realsense 驱动没有订阅者时不发布图像，平时不占带宽。
// 相机中途被拔掉再插回时，驱动重新创建 publisher，订阅会自动重新匹配，录制不中断。
class TopicRecorderComponent : public rclcpp::Node {
 public:
  explicit TopicRecorderComponent(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~TopicRecorderComponent() override;

 private:
  struct TopicSpec {
    std::string name;
    std::string type;
    rclcpp::QoS qos;
  };

  void onRecordRequest(
      const std::shared_ptr<oak_interfaces::srv::Record::Request> request,
      std::shared_ptr<oak_interfaces::srv::Record::Response> response);
  void startRecording(const std::string& bag_path, double duration_s);
  void stopRecording();
  void onMessage(
      const TopicSpec& spec,
      std::shared_ptr<const rclcpp::SerializedMessage> message,
      const rclcpp::MessageInfo& info);
  std::string nextBagPath() const;
  std::string namedBagPath(const std::string& bag_name) const;

  // 参数
  std::string bag_root_;
  std::string bag_prefix_;
  std::string storage_id_;
  double max_duration_s_ = 600.0;
  std::vector<TopicSpec> topics_;

  rclcpp::Service<oak_interfaces::srv::Record>::SharedPtr service_;

  // 录制状态：只在 executor 线程（service / timer 回调）里改。
  bool recording_ = false;
  std::string bag_path_;
  rclcpp::TimerBase::SharedPtr stop_timer_;
  std::vector<rclcpp::GenericSubscription::SharedPtr> subscriptions_;

  // 订阅回调与收尾可能不在同一线程（多线程 executor 时），writer 与计数统一加锁。
  std::mutex writer_mutex_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;
  std::map<std::string, std::uint64_t> counts_;
};

}  // namespace realsense_ros2
