#include "oak_ffc_camera_imu/imu_correction.h"

#include <stdexcept>

#include "yaml-cpp/yaml.h"

namespace oak_ffc_camera_imu {
namespace {

std::array<double, 3> readVector3(const YAML::Node& node, const std::string& name) {
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::runtime_error(name + " must be a 3-element sequence");
  }

  return {node[0].as<double>(), node[1].as<double>(), node[2].as<double>()};
}

std::array<std::array<double, 3>, 3> readMatrix3x3(const YAML::Node& node, const std::string& name) {
  if (!node || !node.IsSequence() || node.size() != 3) {
    throw std::runtime_error(name + " must be a 3x3 sequence");
  }

  std::array<std::array<double, 3>, 3> matrix{};
  for (std::size_t row = 0; row < 3; ++row) {
    matrix[row] = readVector3(node[row], name + " row");
  }
  return matrix;
}

}  // namespace

ImuCorrection ImuCorrection::identity() {
  return ImuCorrection{};
}

ImuCorrection ImuCorrection::loadFromYaml(const std::string& path) {
  if (path.empty()) {
    return identity();
  }

  const auto root = YAML::LoadFile(path);
  ImuCorrection correction;
  correction.enabled_ = true;

  if (root["accelerometer"]) {
    const auto accelerometer = root["accelerometer"];
    correction.accel_bias_ = readVector3(accelerometer["bias"], "accelerometer.bias");
    correction.accel_correction_ =
        readMatrix3x3(accelerometer["correction_matrix"], "accelerometer.correction_matrix");
  } else {
    correction.accel_bias_ = readVector3(root["bias"], "bias");
    correction.accel_correction_ = readMatrix3x3(root["correction_matrix"], "correction_matrix");
  }

  if (root["gyroscope"]) {
    correction.gyro_bias_ = readVector3(root["gyroscope"]["bias"], "gyroscope.bias");
  } else if (root["gyro_bias_static_mean"]) {
    correction.gyro_bias_ = readVector3(root["gyro_bias_static_mean"], "gyro_bias_static_mean");
  }

  return correction;
}

bool ImuCorrection::enabled() const {
  return enabled_;
}

void ImuCorrection::apply(sensor_msgs::msg::Imu& msg) const {
  if (!enabled_) {
    return;
  }

  const std::array<double, 3> accel = {
      msg.linear_acceleration.x - accel_bias_[0],
      msg.linear_acceleration.y - accel_bias_[1],
      msg.linear_acceleration.z - accel_bias_[2],
  };

  msg.linear_acceleration.x = accel_correction_[0][0] * accel[0] +
                              accel_correction_[0][1] * accel[1] +
                              accel_correction_[0][2] * accel[2];
  msg.linear_acceleration.y = accel_correction_[1][0] * accel[0] +
                              accel_correction_[1][1] * accel[1] +
                              accel_correction_[1][2] * accel[2];
  msg.linear_acceleration.z = accel_correction_[2][0] * accel[0] +
                              accel_correction_[2][1] * accel[1] +
                              accel_correction_[2][2] * accel[2];

  msg.angular_velocity.x -= gyro_bias_[0];
  msg.angular_velocity.y -= gyro_bias_[1];
  msg.angular_velocity.z -= gyro_bias_[2];
}

}  // namespace oak_ffc_camera_imu
