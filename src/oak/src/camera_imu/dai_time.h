#pragma once

#include <chrono>
#include <mutex>

#include "depthai/depthai.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/time.hpp"

namespace oak_ffc_camera_imu {

// Converts DepthAI device timestamps (dai::Clock == std::chrono::steady_clock,
// already synced to the host by the DepthAI library) into ROS time.
//
// A single instance is shared between the image and IMU publishers so that both
// sensor streams use the exact same time base. This is required for VIO:
// stamping messages with node->now() on the host receive-side breaks
// camera-IMU alignment and, because DepthAI delivers IMU packets in batches,
// produces near-duplicate / non-monotonic IMU stamps that crash OpenVINS'
// propagator (dt_summed assertion).
//
// The offset between the steady clock and ROS time is captured once, lazily, on
// the first converted timestamp. Every subsequent stamp is
//   ros_time = ros_base + (device_ts - steady_base)
// which preserves the device's true inter-sample intervals and monotonicity.
class DaiTimeConverter {
 public:
  rclcpp::Time toRosTime(
      const std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>& device_ts,
      const rclcpp::Clock::SharedPtr& ros_clock) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
      steady_base_ = device_ts;
      ros_base_ = ros_clock->now();
      initialized_ = true;
    }
    const auto delta = device_ts - steady_base_;
    const auto delta_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(delta).count();
    return ros_base_ + rclcpp::Duration(std::chrono::nanoseconds(delta_ns));
  }

 private:
  std::mutex mutex_;
  bool initialized_ = false;
  std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration> steady_base_{};
  rclcpp::Time ros_base_{0, 0, RCL_ROS_TIME};
};

}  // namespace oak_ffc_camera_imu
