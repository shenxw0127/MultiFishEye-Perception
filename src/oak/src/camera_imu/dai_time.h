#pragma once

#include <chrono>
#include <cstdint>
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
// The offset between the host steady clock and ROS time is sampled once, lazily,
// directly from the two clocks (NOT from the first frame's arrival time):
//   ros_time = device_ts + (ros_now - steady_now)
// This preserves the device's true inter-sample intervals and monotonicity, and
// keeps stamps at the actual capture instant. The old version paired the first
// frame's device_ts with ros_clock->now() at *arrival*, which baked the pipeline
// latency (tens of ms) into every stamp and put OAK behind other sensors (e.g.
// RealSense, whose global-time stamps are true capture times on the system clock).
class DaiTimeConverter {
 public:
  rclcpp::Time toRosTime(
      const std::chrono::time_point<std::chrono::steady_clock, std::chrono::steady_clock::duration>& device_ts,
      const rclcpp::Clock::SharedPtr& ros_clock) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!initialized_) {
      offset_ns_ = sampleOffsetNs(ros_clock);
      initialized_ = true;
    }
    const auto device_ns =
        std::chrono::duration_cast<std::chrono::nanoseconds>(device_ts.time_since_epoch()).count();
    return rclcpp::Time(device_ns + offset_ns_, RCL_ROS_TIME);
  }

 private:
  // ros_now - steady_now, taking the tightest of a few bracketed reads so a
  // preemption between the two clock reads does not skew the offset.
  static int64_t sampleOffsetNs(const rclcpp::Clock::SharedPtr& ros_clock) {
    int64_t best_offset = 0;
    int64_t best_window = INT64_MAX;
    for (int i = 0; i < 10; ++i) {
      const auto before = std::chrono::steady_clock::now();
      const int64_t ros_ns = ros_clock->now().nanoseconds();
      const auto after = std::chrono::steady_clock::now();
      const int64_t window = std::chrono::duration_cast<std::chrono::nanoseconds>(after - before).count();
      if (window < best_window) {
        const int64_t mid_ns =
            std::chrono::duration_cast<std::chrono::nanoseconds>((before + (after - before) / 2).time_since_epoch())
                .count();
        best_window = window;
        best_offset = ros_ns - mid_ns;
      }
    }
    return best_offset;
  }

  std::mutex mutex_;
  bool initialized_ = false;
  int64_t offset_ns_ = 0;
};

}  // namespace oak_ffc_camera_imu
