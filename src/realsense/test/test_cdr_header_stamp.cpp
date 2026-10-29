#include <gtest/gtest.h>

#include <cstdint>
#include <vector>

#include "builtin_interfaces/msg/time.hpp"
#include "rclcpp/serialization.hpp"
#include "rclcpp/serialized_message.hpp"
#include "recorder/cdr_header_stamp.h"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"

namespace realsense_ros2 {
namespace {

// 用真实的 rclcpp 序列化结果测，保证布局假设和 rmw 实际输出一致。
template <typename Msg>
std::optional<std::int64_t> stampOf(const Msg& msg) {
  rclcpp::Serialization<Msg> serialization;
  rclcpp::SerializedMessage serialized;
  serialization.serialize_message(&msg, &serialized);
  const auto& raw = serialized.get_rcl_serialized_message();
  return readLeadingHeaderStamp(raw.buffer, raw.buffer_length);
}

TEST(CdrHeaderStamp, ImageRoundTrip) {
  sensor_msgs::msg::Image msg;
  msg.header.stamp.sec = 1790000000;
  msg.header.stamp.nanosec = 123456789;
  msg.header.frame_id = "d435i_color_optical_frame";
  msg.width = 4;
  msg.height = 2;
  msg.data.resize(8);
  EXPECT_EQ(stampOf(msg), std::int64_t{1790000000} * 1000000000 + 123456789);
}

TEST(CdrHeaderStamp, ImuRoundTrip) {
  sensor_msgs::msg::Imu msg;
  msg.header.stamp.sec = 42;
  msg.header.stamp.nanosec = 7;
  EXPECT_EQ(stampOf(msg), std::int64_t{42} * 1000000000 + 7);
}

TEST(CdrHeaderStamp, ZeroStampIsInvalid) {
  sensor_msgs::msg::Imu msg;  // 默认 stamp = 0
  EXPECT_EQ(stampOf(msg), std::nullopt);
}

TEST(CdrHeaderStamp, BigEndianEncapsulation) {
  // [0x00 0x00 opt opt] 大端：sec = 0x00000100 = 256，nanosec = 5
  const std::vector<std::uint8_t> data = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
                                          0x00, 0x00, 0x00, 0x05, 0xff, 0xff};
  EXPECT_EQ(readLeadingHeaderStamp(data.data(), data.size()), std::int64_t{256} * 1000000000 + 5);
}

TEST(CdrHeaderStamp, RejectsTruncatedAndNull) {
  const std::vector<std::uint8_t> data = {0x00, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00};
  EXPECT_EQ(readLeadingHeaderStamp(data.data(), data.size()), std::nullopt);
  EXPECT_EQ(readLeadingHeaderStamp(nullptr, 64), std::nullopt);
}

TEST(CdrHeaderStamp, RejectsOutOfRangeNanosec) {
  const std::vector<std::uint8_t> data = {0x00, 0x01, 0x00, 0x00, 0x05, 0x00, 0x00, 0x00,
                                          0x00, 0xca, 0x9a, 0x3b};  // nanosec = 1e9
  EXPECT_EQ(readLeadingHeaderStamp(data.data(), data.size()), std::nullopt);
}

TEST(CdrHeaderStamp, OnlyHeaderLeadingTypes) {
  EXPECT_TRUE(hasLeadingHeader("sensor_msgs/msg/Image"));
  EXPECT_TRUE(hasLeadingHeader("sensor_msgs/msg/Imu"));
  EXPECT_FALSE(hasLeadingHeader("tf2_msgs/msg/TFMessage"));
  EXPECT_FALSE(hasLeadingHeader("realsense2_camera_msgs/msg/Extrinsics"));
}

}  // namespace
}  // namespace realsense_ros2
