#pragma once

#include <chrono>
#include <cstdint>
#include <map>
#include <optional>
#include <string>

namespace oak_ffc_camera_imu {

// 各插座的累计帧数，按插座名排序。
using FrameCounts = std::map<std::string, std::uint64_t>;
// 各插座的帧率；nullopt 表示这一段算不出来（刚连上或刚重连）。
using FpsBySocket = std::map<std::string, std::optional<double>>;

// 两次累计计数之间的速率。没有上次计数、上次还没收到过数据（首帧落在区间中途，算出来偏低）、
// 计数比上次小（重连后清零）或间隔非正时为 nullopt。
std::optional<double> computeRate(std::optional<std::uint64_t> previous, std::uint64_t current, double seconds);

// current 里每个插座的帧率，规则同 computeRate。
FpsBySocket computeFps(const FrameCounts& previous, const FrameCounts& current, double seconds);

// 单行状态文本，不含光标控制符，形如
//   [oak fps] CAM_A 30.0  CAM_B 30.0 /30  IMU 200 /200
// 斜杠后是目标频率。cameras 为空表示设备未连接。color 为 true 时，低于目标 95% 的读数标红。
std::string formatFpsLine(
    const std::string& label,
    const FpsBySocket& cameras,
    int camera_target_fps,
    std::optional<double> imu_hz,
    int imu_target_hz,
    bool color);

// 在控制终端上单行刷新各路相机帧率和 IMU 频率。直接写 /dev/tty 而不是 stdout：ros2 launch 按整行转发
// 子进程输出，不带换行的 \r 刷新经它显示不出来。没有控制终端（如 systemd 服务）时 available() 为 false。
class FpsDisplay {
 public:
  FpsDisplay(std::string label, int camera_target_fps, int imu_target_hz);
  ~FpsDisplay();

  FpsDisplay(const FpsDisplay&) = delete;
  FpsDisplay& operator=(const FpsDisplay&) = delete;

  bool available() const { return tty_fd_ >= 0; }
  // 用最新累计计数重绘一次；速率按与上次调用之间的实际间隔计算。imu_packets 为 nullopt 表示 IMU 未连接。
  void update(const FrameCounts& camera_frames, std::optional<std::uint64_t> imu_packets);

 private:
  void write(const std::string& text);

  std::string label_;
  int camera_target_fps_;
  int imu_target_hz_;
  int tty_fd_ = -1;
  FrameCounts last_camera_frames_;
  std::optional<std::uint64_t> last_imu_packets_;
  std::chrono::steady_clock::time_point last_time_;
};

}  // namespace oak_ffc_camera_imu
