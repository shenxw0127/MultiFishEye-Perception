#pragma once

#include <atomic>
#include <condition_variable>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "depthai/depthai.hpp"
#include "camera_imu/config.h"
#include "camera_imu/dai_time.h"
#include "camera_imu/fps_display.h"
#include "camera_imu/image_publisher.h"
#include "camera_imu/imu_publisher.h"
#include "camera_imu/pipeline_builder.h"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"

namespace oak_ffc_camera_imu {

// 设备生命周期由 supervisor 线程独占：连接 -> 监视 -> 断连后清理 -> 按间隔重试。
// ROS publisher 在构造时一次建好并跨重连复用，订阅方无需感知热插拔。
class OakFfcCameraImuComponent : public rclcpp::Node {
 public:
  explicit OakFfcCameraImuComponent(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
  ~OakFfcCameraImuComponent() override;

 private:
  void superviseDevice();
  void connectDevice();
  // 探测本次连接实际出帧的插座（已与 cam_board_sockets 取交集）。会占用并关闭设备的一次启动，
  // device_id 返回被探测设备的 MxId，供随后重开同一台。
  std::vector<std::string> resolveStreamingSockets(const DriverConfig& config, std::string& device_id);
  void disconnectDevice();
  bool deviceHealthy() const;
  // 可被析构提前唤醒的 sleep，返回 false 表示该退出了。
  bool waitFor(std::chrono::milliseconds duration);
  void sendCameraControls(const DriverConfig& config);
  rcl_interfaces::msg::SetParametersResult onParametersChanged(const std::vector<rclcpp::Parameter>& parameters);

  // config_ 启动后只会改 camera control 字段（见 onParametersChanged），
  // 重连时用它重建 pipeline，因此断连期间改的曝光等参数在重连后自动生效。
  mutable std::mutex config_mutex_;
  DriverConfig config_;

  std::shared_ptr<DaiTimeConverter> time_converter_;
  std::unique_ptr<SyncedImagePublisher> image_publisher_;
  std::unique_ptr<ImuPublisher> imu_publisher_;
  // fps_display 关闭或没有控制终端时都为空。
  std::unique_ptr<FpsDisplay> fps_display_;
  rclcpp::TimerBase::SharedPtr fps_display_timer_;

  // 仅 supervisor 线程读写 device_；control queue 另受 control_mutex_ 保护，因为参数回调在 executor 线程发控制。
  std::unique_ptr<dai::Device> device_;
  std::mutex control_mutex_;
  std::map<std::string, std::shared_ptr<dai::DataInputQueue>> camera_control_queues_;

  std::atomic_bool running_{true};
  std::mutex wake_mutex_;
  std::condition_variable wake_cv_;
  std::thread supervisor_thread_;

  OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

}  // namespace oak_ffc_camera_imu
