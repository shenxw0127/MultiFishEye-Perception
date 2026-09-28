#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "oak_interfaces/srv/record.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rosbag2_cpp/writer.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace oak_ffc_camera_imu {

// 按需定时录制：收到 /oak/record 请求后录 duration_s 秒 rosbag，到时自动收尾。
//
// 只在录制期间订阅 raw 图像和 IMU（同容器 intra-process，零拷贝），平时不影响驱动的
// lazy_publisher。图像回调只入有界队列，JPEG 压缩在工作线程里做，不阻塞 executor；
// 队列满时丢帧并计数，保证录制不会拖垮机载端。
//
// 图像按 record_hz 抽帧（相机仍按 fps 采集，只是少写盘）：以绝对时间 1/record_hz 为格子，
// 每格只留第一帧。4 路同步组共享同一时间戳，所以抽到的总是同一组；和其它相机的录制也落在
// 同一套时间格上，便于离线配对。
//
// 所有话题按 header.stamp 去重（只收严格递增的）：同进程里有 foxglove_bridge 这类
// 不走 intra-process 的订阅者时，驱动会同时走 intra-process 和 DDS 发布，本订阅两边都会收到，
// rclcpp 的 gid 去重实测不可靠（旧版录出 60Hz，每帧两份）。
class BagRecorderComponent : public rclcpp::Node {
 public:
  explicit BagRecorderComponent(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~BagRecorderComponent() override;

 private:
  struct EncodeJob {
    std::string topic;
    sensor_msgs::msg::Image::ConstSharedPtr image;
  };

  void onRecordRequest(
      const std::shared_ptr<oak_interfaces::srv::Record::Request> request,
      std::shared_ptr<oak_interfaces::srv::Record::Response> response);
  void startRecording(const std::string& bag_path, double duration_s);
  void stopRecording();
  void onImage(const std::string& output_topic, sensor_msgs::msg::Image::ConstSharedPtr image);
  // 去重 + 抽帧判定；period_ns <= 0 时只去重。只在 executor 线程里调用。
  bool acceptStamp(const std::string& topic, const builtin_interfaces::msg::Time& stamp, int64_t period_ns);
  void onImu(sensor_msgs::msg::Imu::ConstSharedPtr imu);
  void encodeWorker();
  std::string nextBagPath() const;
  std::string namedBagPath(const std::string& bag_name) const;

  // 参数
  std::string bag_root_;
  std::string bag_prefix_;
  std::string storage_id_;
  int jpeg_quality_ = 90;
  std::size_t max_queue_ = 60;
  double max_duration_s_ = 600.0;
  int64_t record_period_ns_ = 0;  // 图像抽帧间隔，0 = 不抽帧
  std::vector<std::string> cam_board_sockets_;
  std::string imu_topic_;

  rclcpp::Service<oak_interfaces::srv::Record>::SharedPtr service_;

  // 录制状态：只在 executor 线程（service / timer 回调）里改。
  bool recording_ = false;
  std::string bag_path_;
  rclcpp::TimerBase::SharedPtr stop_timer_;
  std::vector<rclcpp::SubscriptionBase::SharedPtr> subscriptions_;
  // 每个话题最后接受的 时间戳 / 时间格，用于去重和抽帧
  std::map<std::string, int64_t> last_stamp_ns_;
  std::map<std::string, int64_t> last_bucket_;

  // writer 被 IMU 回调和编码线程并发写，统一用 writer_mutex_ 串行化。
  std::mutex writer_mutex_;
  std::unique_ptr<rosbag2_cpp::Writer> writer_;

  // 编码队列
  std::mutex queue_mutex_;
  std::condition_variable queue_cv_;
  std::condition_variable idle_cv_;
  std::deque<EncodeJob> queue_;
  std::size_t in_flight_ = 0;
  bool workers_running_ = true;
  std::vector<std::thread> workers_;

  std::atomic<std::uint64_t> images_written_{0};
  std::atomic<std::uint64_t> imu_written_{0};
  std::atomic<std::uint64_t> images_dropped_{0};
  std::uint64_t duplicates_skipped_ = 0;
};

}  // namespace oak_ffc_camera_imu
