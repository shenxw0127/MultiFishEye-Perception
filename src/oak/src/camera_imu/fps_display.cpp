#include "camera_imu/fps_display.h"

#include <fcntl.h>
#include <unistd.h>

#include <cstdio>
#include <utility>

namespace oak_ffc_camera_imu {
namespace {

constexpr const char* kRed = "\033[31m";
constexpr const char* kReset = "\033[0m";
// 低于目标帧率这个比例才标红，避免时钟抖动造成的 29.9 也闪红。
constexpr double kLowFpsRatio = 0.95;

// 按 format 格式化读数，未知时用等宽的 unknown 占位；低于目标时按需标红。
std::string formatRate(std::optional<double> rate, int target, const char* format, const char* unknown, bool color) {
  if (!rate) {
    return unknown;
  }
  char value[16];
  std::snprintf(value, sizeof(value), format, *rate);
  const bool low = *rate < kLowFpsRatio * target;
  return color && low ? std::string(kRed) + value + kReset : std::string(value);
}

}  // namespace

std::optional<double> computeRate(std::optional<std::uint64_t> previous, std::uint64_t current, double seconds) {
  if (seconds <= 0.0 || !previous || *previous == 0 || current < *previous) {
    return std::nullopt;
  }
  return static_cast<double>(current - *previous) / seconds;
}

FpsBySocket computeFps(const FrameCounts& previous, const FrameCounts& current, double seconds) {
  FpsBySocket fps;
  for (const auto& entry : current) {
    const auto previous_it = previous.find(entry.first);
    const auto previous_count =
        previous_it == previous.end() ? std::nullopt : std::optional<std::uint64_t>(previous_it->second);
    fps[entry.first] = computeRate(previous_count, entry.second, seconds);
  }
  return fps;
}

std::string formatFpsLine(
    const std::string& label,
    const FpsBySocket& cameras,
    int camera_target_fps,
    std::optional<double> imu_hz,
    int imu_target_hz,
    bool color) {
  std::string line = "[" + label + " fps]";
  if (cameras.empty()) {
    return line + " waiting for device...";
  }
  for (const auto& entry : cameras) {
    line += (entry.first == cameras.begin()->first ? " " : "  ") + entry.first + " " +
            formatRate(entry.second, camera_target_fps, "%4.1f", " -- ", color);
  }
  line += " /" + std::to_string(camera_target_fps);
  // IMU 只看整数 Hz 就够了，省下的宽度让 4 路相机 + IMU 能放进 80 列。
  return line + "  IMU " + formatRate(imu_hz, imu_target_hz, "%3.0f", " --", color) + " /" +
         std::to_string(imu_target_hz);
}

FpsDisplay::FpsDisplay(std::string label, int camera_target_fps, int imu_target_hz)
    : label_(std::move(label)),
      camera_target_fps_(camera_target_fps),
      imu_target_hz_(imu_target_hz),
      tty_fd_(::open("/dev/tty", O_WRONLY | O_NOCTTY | O_CLOEXEC)),
      last_time_(std::chrono::steady_clock::now()) {}

FpsDisplay::~FpsDisplay() {
  if (tty_fd_ >= 0) {
    // 留下最后一次读数，把光标移到下一行，后续退出日志不会盖在上面。
    write("\n");
  }
  if (tty_fd_ >= 0) {
    ::close(tty_fd_);
  }
}

void FpsDisplay::update(const FrameCounts& camera_frames, std::optional<std::uint64_t> imu_packets) {
  if (tty_fd_ < 0) {
    return;
  }
  const auto now = std::chrono::steady_clock::now();
  const double seconds = std::chrono::duration<double>(now - last_time_).count();
  const auto cameras = computeFps(last_camera_frames_, camera_frames, seconds);
  const auto imu_hz = imu_packets ? computeRate(last_imu_packets_, *imu_packets, seconds) : std::nullopt;
  last_camera_frames_ = camera_frames;
  last_imu_packets_ = imu_packets;
  last_time_ = now;
  // \r\033[K 清掉当前行再画；结尾 \r 把光标留在行首，其他日志打出来时会从行首覆盖这行，
  // 而不是接在状态行后面。
  write("\r\033[K" + formatFpsLine(label_, cameras, camera_target_fps_, imu_hz, imu_target_hz_, true) + "\r");
}

void FpsDisplay::write(const std::string& text) {
  // 终端关掉（SSH 断开）后写会失败，此后不再显示，驱动照常运行。
  if (::write(tty_fd_, text.data(), text.size()) < 0) {
    ::close(tty_fd_);
    tty_fd_ = -1;
  }
}

}  // namespace oak_ffc_camera_imu
