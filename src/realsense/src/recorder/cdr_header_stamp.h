#pragma once

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string_view>

namespace realsense_ros2 {

// 第一个字段是 std_msgs/Header 的消息类型（header.stamp 紧跟在 CDR 头后面，可以不反序列化直接读）。
// 新增的话题类型如果也是 header 打头，加到这里即可；不在表里的类型一律用接收时间。
inline bool hasLeadingHeader(std::string_view type) {
  constexpr std::string_view kTypes[] = {
      "sensor_msgs/msg/Image",
      "sensor_msgs/msg/CameraInfo",
      "sensor_msgs/msg/Imu",
      "geometry_msgs/msg/PoseStamped",
  };
  for (const auto candidate : kTypes) {
    if (type == candidate) {
      return true;
    }
  }
  return false;
}

// 从序列化 CDR 数据里读出开头的 header.stamp（纳秒），不是合法 stamp 时返回 nullopt。
//
// 布局：4 字节封装头 [0, flag, opt, opt]，随后是 int32 sec、uint32 nanosec（4 字节对齐，
// 正好紧跟在封装头之后）。flag 的最低位表示字节序（0 = 大端，1 = 小端；XCDR1/XCDR2 都适用）。
// ROS 消息都是 final 结构，Header 作为第一个成员前面没有 DHEADER。
inline std::optional<std::int64_t> readLeadingHeaderStamp(const std::uint8_t* data, std::size_t size) {
  constexpr std::size_t kEncapsulation = 4;
  constexpr std::size_t kStampBytes = 8;
  if (data == nullptr || size < kEncapsulation + kStampBytes) {
    return std::nullopt;
  }
  const bool little_endian = (data[1] & 0x01) != 0;
  auto read_u32 = [&](std::size_t offset) {
    const std::uint8_t* p = data + offset;
    return little_endian
               ? (std::uint32_t{p[0]} | std::uint32_t{p[1]} << 8 | std::uint32_t{p[2]} << 16 |
                  std::uint32_t{p[3]} << 24)
               : (std::uint32_t{p[3]} | std::uint32_t{p[2]} << 8 | std::uint32_t{p[1]} << 16 |
                  std::uint32_t{p[0]} << 24);
  };
  const auto sec = static_cast<std::int32_t>(read_u32(kEncapsulation));
  const auto nanosec = read_u32(kEncapsulation + 4);
  // 没填 stamp（全 0）或 nanosec 越界，都视为无效，让调用方退回接收时间。
  if (sec <= 0 || nanosec >= 1000000000u) {
    return std::nullopt;
  }
  return std::int64_t{sec} * 1000000000 + std::int64_t{nanosec};
}

}  // namespace realsense_ros2
