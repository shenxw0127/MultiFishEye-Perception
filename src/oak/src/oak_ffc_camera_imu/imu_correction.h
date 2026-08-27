#pragma once

#include <array>
#include <string>

#include "sensor_msgs/msg/imu.hpp"

namespace oak_ffc_camera_imu {

class ImuCorrection {
 public:
  static ImuCorrection identity();
  static ImuCorrection loadFromYaml(const std::string& path);

  bool enabled() const;
  void apply(sensor_msgs::msg::Imu& msg) const;

 private:
  bool enabled_ = false;
  std::array<double, 3> accel_bias_{0.0, 0.0, 0.0};
  std::array<double, 3> gyro_bias_{0.0, 0.0, 0.0};
  std::array<std::array<double, 3>, 3> accel_correction_{{
      {{1.0, 0.0, 0.0}},
      {{0.0, 1.0, 0.0}},
      {{0.0, 0.0, 1.0}},
  }};
};

}  // namespace oak_ffc_camera_imu
