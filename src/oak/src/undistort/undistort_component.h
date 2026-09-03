#pragma once

#include <string>

#include "opencv2/core.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "sensor_msgs/msg/image.hpp"

namespace oak_ffc_camera_imu {

// 鱼眼去畸变组件：订阅 raw sensor_msgs/Image，用 cv_bridge::toCvShare 零拷贝
// 引用消息内存（无需解码），用 Kannala-Brandt (equidistant) 模型去畸变，
// 输出针孔模型的 raw sensor_msgs/Image（不压缩）。
// 内参从 Kalibr camchain yaml 读取（config/camera 下）。
class FisheyeUndistortComponent : public rclcpp::Node {
 public:
  explicit FisheyeUndistortComponent(const rclcpp::NodeOptions& options);

 private:
  void loadCalibration(const std::string& camchain_path);
  void buildRemap();
  void callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);

  std::string output_frame_id_;

  // 标定参数
  cv::Matx33d K_;          // 内参矩阵
  cv::Vec4d D_;            // equidistant 畸变系数 [k1,k2,k3,k4]
  cv::Size image_size_;    // 标定分辨率 [width, height]

  // 预计算的 remap 查找表
  cv::Mat map1_;
  cv::Mat map2_;

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
};

}  // namespace oak_ffc_camera_imu
