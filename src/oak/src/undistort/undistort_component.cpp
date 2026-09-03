#include "undistort/undistort_component.h"

#include <stdexcept>
#include <vector>

#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/image_encodings.hpp"
#include "yaml-cpp/yaml.h"

namespace oak_ffc_camera_imu {
namespace {

constexpr const char* kEncodingNv12 = "nv12";
constexpr const char* kEncodingNv21 = "nv21";

}  // namespace

FisheyeUndistortComponent::FisheyeUndistortComponent(const rclcpp::NodeOptions& options)
    : rclcpp::Node("fisheye_undistort", options) {
  const auto camchain_path = declare_parameter<std::string>("camchain_path", "");
  const auto input_topic = declare_parameter<std::string>("input_topic", "image");
  const auto output_topic = declare_parameter<std::string>("output_topic", "image_rect");
  output_frame_id_ = declare_parameter<std::string>("output_frame_id", "");

  if (camchain_path.empty()) {
    throw std::runtime_error("camchain_path parameter is required");
  }
  loadCalibration(camchain_path);
  buildRemap();

  pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic, rclcpp::SensorDataQoS());
  sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&FisheyeUndistortComponent::callback, this, std::placeholders::_1));

  RCLCPP_INFO(
      get_logger(),
      "Fisheye undistort ready: %s -> %s (fx=%.2f fy=%.2f cx=%.2f cy=%.2f, %dx%d)",
      input_topic.c_str(),
      output_topic.c_str(),
      K_(0, 0),
      K_(1, 1),
      K_(0, 2),
      K_(1, 2),
      image_size_.width,
      image_size_.height);
}

void FisheyeUndistortComponent::loadCalibration(const std::string& camchain_path) {
  const auto root = YAML::LoadFile(camchain_path);
  if (!root["cam0"]) {
    throw std::runtime_error("camchain missing cam0: " + camchain_path);
  }
  const auto cam = root["cam0"];

  const auto model = cam["distortion_model"].as<std::string>();
  if (model != "equidistant") {
    throw std::runtime_error("unsupported distortion_model '" + model + "' (expected equidistant)");
  }

  const auto intrinsics = cam["intrinsics"].as<std::vector<double>>();
  if (intrinsics.size() != 4) {
    throw std::runtime_error("intrinsics must have 4 values [fx, fy, cx, cy]");
  }
  K_ = cv::Matx33d::eye();
  K_(0, 0) = intrinsics[0];  // fx
  K_(1, 1) = intrinsics[1];  // fy
  K_(0, 2) = intrinsics[2];  // cx
  K_(1, 2) = intrinsics[3];  // cy

  const auto distortion = cam["distortion_coeffs"].as<std::vector<double>>();
  if (distortion.size() != 4) {
    throw std::runtime_error("equidistant distortion_coeffs must have 4 values [k1..k4]");
  }
  D_ = cv::Vec4d(distortion[0], distortion[1], distortion[2], distortion[3]);

  const auto resolution = cam["resolution"].as<std::vector<int>>();
  if (resolution.size() != 2) {
    throw std::runtime_error("resolution must have 2 values [width, height]");
  }
  image_size_ = cv::Size(resolution[0], resolution[1]);
}

void FisheyeUndistortComponent::buildRemap() {
  // 裁剪视场：新投影矩阵直接用原内参 K，等价于 cv::fisheye::undistortImage 默认行为。
  cv::fisheye::initUndistortRectifyMap(
      K_,
      D_,
      cv::Matx33d::eye(),
      K_,
      image_size_,
      CV_16SC2,
      map1_,
      map2_);
}

void FisheyeUndistortComponent::callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  // 取原始图像。规整格式 (BGR8/RGB8/MONO8) 用 toCvShare 零拷贝直接引用消息内存，
  // 无需解码；NV12 (彩色相机 ISP 输出) 需一次轻量 cvtColor 转 BGR 才能 remap。
  cv::Mat image;
  std::string out_encoding;
  try {
    if (msg->encoding == kEncodingNv12 || msg->encoding == kEncodingNv21) {
      // NV12/NV21: YUV420 半平面，高度为 H*3/2 的单通道缓冲；无法直接 remap
      const cv::Mat yuv(
          msg->height * 3 / 2,
          msg->width,
          CV_8UC1,
          const_cast<uint8_t*>(msg->data.data()),
          msg->step);
      const int code = msg->encoding == kEncodingNv12 ? cv::COLOR_YUV2BGR_NV12 : cv::COLOR_YUV2BGR_NV21;
      cv::cvtColor(yuv, image, code);
      out_encoding = sensor_msgs::image_encodings::BGR8;
    } else {
      // 零拷贝共享消息内存
      auto shared = cv_bridge::toCvShare(msg);
      image = shared->image;
      out_encoding = msg->encoding;
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "toCvShare failed (encoding=%s): %s", msg->encoding.c_str(), e.what());
    return;
  }

  if (image.size() != image_size_) {
    RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        5000,
        "Image size %dx%d does not match calibration %dx%d; skipping",
        image.cols,
        image.rows,
        image_size_.width,
        image_size_.height);
    return;
  }

  cv::Mat rectified;
  cv::remap(image, rectified, map1_, map2_, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

  sensor_msgs::msg::Image out;
  out.header = msg->header;
  if (!output_frame_id_.empty()) {
    out.header.frame_id = output_frame_id_;
  }
  out.height = rectified.rows;
  out.width = rectified.cols;
  out.encoding = out_encoding;
  out.is_bigendian = false;
  out.step = static_cast<uint32_t>(rectified.cols) * rectified.elemSize();
  out.data.assign(rectified.datastart, rectified.dataend);
  pub_->publish(out);
}

}  // namespace oak_ffc_camera_imu

RCLCPP_COMPONENTS_REGISTER_NODE(oak_ffc_camera_imu::FisheyeUndistortComponent)
