#pragma once

#include <map>
#include <string>
#include <tuple>
#include <vector>

#include "camera_imu/socket_list.h"
#include "depthai/depthai.hpp"
#include "rclcpp/node.hpp"

namespace oak_ffc_camera_imu {

enum class CameraType {
  Color,
  Mono,
};

struct CameraSocketConfig {
  dai::CameraBoardSocket socket;
  std::string image_topic;
};

struct ResolutionConfig {
  int width = 0;
  int height = 0;
  dai::node::ColorCamera::Properties::SensorResolution color_resolution;
  dai::node::MonoCamera::Properties::SensorResolution mono_resolution;
  bool supports_color = false;
  bool supports_mono = false;
  // 色彩路径专用：ISP 硬件缩放比例，套在 color_resolution 之上。depthai 的
  // ColorCamera 没有 400p 这档 sensor 分辨率，只能取 800p 再由 ISP 缩一半，
  // 这样裁的是像素不是视场，鱼眼 FOV 保持不变。1/1 表示不缩放。
  // MonoCamera 没有 ISP 缩放，原生就有 400p，因此这两个字段对 mono 无效。
  int isp_scale_num = 1;
  int isp_scale_den = 1;
};

struct DriverConfig {
  std::string tf_prefix = "oak";
  std::string camera_name = "ov9782";
  CameraType camera_type = CameraType::Color;
  std::string resolution = "800p";
  int fps = 30;
  bool compressed = true;
  // 允许协商的最高 USB 速率："super"（USB3）或 "high"（强制 USB2，避免 USB3 射频干扰 GNSS）。
  std::string usb_speed = "super";
  // 设备端把图像编成 MJPEG 再过 XLink。USB2 下 4×400p@30 裸 GRAY8 需要 30.7 MB/s，
  // 顶到 USB2 实际上限（实测只能跑 ~24fps），MJPEG 后仅 ~7.5 MB/s，满帧零丢帧。
  // compressed=false 时主机端解码回 mono8/bgr8 再发布，下游接口不变。
  bool xlink_mjpeg = false;
  // 设备端 MJPEG 质量 1..100（xlink_mjpeg 或 compressed 时生效）。
  int jpeg_quality = 95;
  std::string oak_fw_uri;
  std::string imu_config_path;
  bool imu_correction_enabled = true;
  int imu_hz = 200;
  // 显式插座列表，或 {"ALL"}：连设备后按实际插着的相机自动选路（见 socket_list.h）。
  std::vector<std::string> cam_board_sockets = {"CAM_A", "CAM_B", "CAM_C", "CAM_D"};
  int sync_threshold_ms = 50;
  int image_queue_size = 1;
  int imu_queue_size = 50;
  bool lazy_publisher = true;
  // 设备不在或断连后，两次重连尝试之间的间隔。
  int reconnect_interval_ms = 1000;
  // 在启动终端单行刷新各路相机实际帧率（写 /dev/tty，没有终端时自动关闭）。
  bool fps_display = true;
  int manual_exposure_us = 0;
  int manual_iso = 0;
  int exposure_compensation = 0;
  // Caps the exposure time the firmware auto-exposure may pick, so it has to
  // raise ISO instead of lengthening exposure. Only used when auto exposure is
  // active (manual_exposure_us or manual_iso left at 0). 0 disables the cap.
  int auto_exposure_limit_us = 0;
  int manual_wb_kelvin = 0;
  int brightness = 0;
  int contrast = 0;
  int saturation = 0;
  int sharpness = 1;
  int luma_denoise = 1;
  int chroma_denoise = 1;
  int manual_focus = -1;
};

const std::map<std::string, CameraSocketConfig>& cameraSocketOptions();
dai::UsbSpeed usbSpeedFromString(const std::string& value);
// 设备端是否有 MJPEG 编码器：ROS 端要压缩图，或链路上走 MJPEG。
inline bool deviceEncodesMjpeg(const DriverConfig& config) { return config.compressed || config.xlink_mjpeg; }
const std::map<std::string, ResolutionConfig>& resolutionOptions();
// 按 sensor 修正 resolutionOptions() 里的 color sensor 档位 / ISP 缩放，输出尺寸不变。
// AR0234 原生 1920x1200：400p/800p 取 1200p 全幅再由 ISP 缩 1/3、2/3，保证视场不变。
ResolutionConfig resolveResolution(const DriverConfig& config);

void declareParameters(rclcpp::Node& node, const DriverConfig& defaults);
DriverConfig readParameters(rclcpp::Node& node);
void validateConfig(const DriverConfig& config);

std::string toString(CameraType type);
CameraType cameraTypeFromString(const std::string& value);

}  // namespace oak_ffc_camera_imu
