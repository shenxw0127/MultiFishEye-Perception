#include "oak_ffc_camera_imu/mono_image_converter_component.h"

#include <algorithm>
#include <stdexcept>

#include "opencv2/imgproc.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/image_encodings.hpp"

namespace oak_ffc_camera_imu {
namespace {

bool hasValidStorage(const sensor_msgs::msg::Image& msg) {
  return msg.height > 0 && msg.width > 0 && msg.step > 0 &&
         msg.data.size() >= static_cast<std::size_t>(msg.step) * msg.height;
}

void copyMonoRows(const sensor_msgs::msg::Image& src, sensor_msgs::msg::Image& dst) {
  dst.data.resize(static_cast<std::size_t>(src.width) * src.height);
  for (uint32_t row = 0; row < src.height; ++row) {
    const auto* begin = src.data.data() + static_cast<std::size_t>(row) * src.step;
    std::copy(begin, begin + src.width, dst.data.data() + static_cast<std::size_t>(row) * src.width);
  }
}

}  // namespace

MonoImageConverterComponent::MonoImageConverterComponent(const rclcpp::NodeOptions& options)
    : rclcpp::Node("mono_image_converter", options) {
  const auto input_topic = declare_parameter<std::string>("input_topic", "image");
  const auto output_topic = declare_parameter<std::string>("output_topic", "image_mono");
  output_frame_id_ = declare_parameter<std::string>("output_frame_id", "");

  pub_ = create_publisher<sensor_msgs::msg::Image>(output_topic, rclcpp::SensorDataQoS());
  sub_ = create_subscription<sensor_msgs::msg::Image>(
      input_topic,
      rclcpp::SensorDataQoS(),
      std::bind(&MonoImageConverterComponent::callback, this, std::placeholders::_1));
}

void MonoImageConverterComponent::callback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  sensor_msgs::msg::Image out;
  out.header = msg->header;
  if (!output_frame_id_.empty()) {
    out.header.frame_id = output_frame_id_;
  }
  out.height = msg->height;
  out.width = msg->width;
  out.encoding = sensor_msgs::image_encodings::MONO8;
  out.is_bigendian = false;
  out.step = msg->width;

  try {
    if ((msg->encoding == sensor_msgs::image_encodings::MONO8 || msg->encoding == "8UC1") && hasValidStorage(*msg)) {
      copyMonoRows(*msg, out);
      pub_->publish(out);
      return;
    }

    if ((msg->encoding == sensor_msgs::image_encodings::NV12 || msg->encoding == sensor_msgs::image_encodings::NV21) &&
        msg->data.size() >= static_cast<std::size_t>(msg->width) * msg->height) {
      const auto pixel_count = static_cast<std::size_t>(msg->width) * msg->height;
      out.data.assign(msg->data.begin(), msg->data.begin() + static_cast<std::ptrdiff_t>(pixel_count));
      pub_->publish(out);
      return;
    }

    if (hasValidStorage(*msg) &&
        (msg->encoding == sensor_msgs::image_encodings::BGR8 || msg->encoding == sensor_msgs::image_encodings::RGB8 ||
         msg->encoding == sensor_msgs::image_encodings::BGRA8 || msg->encoding == sensor_msgs::image_encodings::RGBA8)) {
      int type = CV_8UC3;
      int code = msg->encoding == sensor_msgs::image_encodings::RGB8 ? cv::COLOR_RGB2GRAY : cv::COLOR_BGR2GRAY;
      if (msg->encoding == sensor_msgs::image_encodings::BGRA8 || msg->encoding == sensor_msgs::image_encodings::RGBA8) {
        type = CV_8UC4;
        code = msg->encoding == sensor_msgs::image_encodings::RGBA8 ? cv::COLOR_RGBA2GRAY : cv::COLOR_BGRA2GRAY;
      }
      cv::Mat color(msg->height, msg->width, type, const_cast<uint8_t*>(msg->data.data()), msg->step);
      cv::Mat gray;
      cv::cvtColor(color, gray, code);
      out.data.assign(gray.datastart, gray.dataend);
      pub_->publish(out);
      return;
    }

    const auto pixel_count = static_cast<std::size_t>(msg->width) * msg->height;
    if (pixel_count > 0 && msg->data.size() >= pixel_count) {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          5000,
          "Input image %s is not directly supported; publishing the first luminance-sized plane as mono8",
          msg->encoding.c_str());
      out.data.assign(msg->data.begin(), msg->data.begin() + static_cast<std::ptrdiff_t>(pixel_count));
      pub_->publish(out);
      return;
    }
  } catch (const std::exception& e) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Failed to convert image to mono8: %s", e.what());
    return;
  }

  RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Cannot convert image to mono8: encoding=%s width=%u height=%u step=%u size=%zu",
      msg->encoding.c_str(),
      msg->width,
      msg->height,
      msg->step,
      msg->data.size());
}

}  // namespace oak_ffc_camera_imu

RCLCPP_COMPONENTS_REGISTER_NODE(oak_ffc_camera_imu::MonoImageConverterComponent)
