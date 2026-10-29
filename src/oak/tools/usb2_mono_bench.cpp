// OAK-FFC 4×OV9782 带宽基准：绕开 ROS，按驱动同样的拓扑（FrameSync + Sync + IMU）建 pipeline，
// 在主机端直接数 XLink 上收到的同步组，报告协商 USB 速率、每秒组数和 payload 吞吐。
//
// 用法：usb2_mono_bench [--usb high|super] [--mode mono|mono-mjpeg|color-nv12]
//                       [--res 400p|800p] [--fps 30] [--sockets 4] [--seconds 20] [--chunk 0]
//                       [--quality 0|1..100|-1(lossless)]
//
//   mono        MonoCamera 原生 GRAY8（厂商 ISP 直接出亮度，不经过彩色）
//   mono-mjpeg  MonoCamera GRAY8 -> 设备端 MJPEG
//   color-nv12  ColorCamera 800p + ISP 1/2 -> video(NV12)，作为对照
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <map>
#include <string>
#include <vector>

#include "depthai/depthai.hpp"

namespace {

struct Args {
  std::string usb = "high";
  std::string mode = "mono";
  std::string res = "400p";
  int fps = 30;
  int sockets = 4;
  int seconds = 20;
  int chunk = 0;
  std::string dump;  // 非空则把第一帧 CAM_A 原样写到该路径
  int quality = 0;  // 0 = 预设默认；1..100 = MJPEG 质量；-1 = 无损
};

Args parseArgs(int argc, char** argv) {
  Args args;
  for (int i = 1; i + 1 < argc; i += 2) {
    const std::string key = argv[i];
    const std::string value = argv[i + 1];
    if (key == "--usb") args.usb = value;
    else if (key == "--mode") args.mode = value;
    else if (key == "--res") args.res = value;
    else if (key == "--fps") args.fps = std::stoi(value);
    else if (key == "--sockets") args.sockets = std::stoi(value);
    else if (key == "--seconds") args.seconds = std::stoi(value);
    else if (key == "--chunk") args.chunk = std::stoi(value);
    else if (key == "--dump") args.dump = value;
    else if (key == "--quality") args.quality = std::stoi(value);
    else {
      std::fprintf(stderr, "unknown arg %s\n", key.c_str());
      std::exit(2);
    }
  }
  return args;
}

const char* usbSpeedName(dai::UsbSpeed speed) {
  switch (speed) {
    case dai::UsbSpeed::LOW: return "LOW (1.5M)";
    case dai::UsbSpeed::FULL: return "FULL (12M)";
    case dai::UsbSpeed::HIGH: return "HIGH / USB2 (480M)";
    case dai::UsbSpeed::SUPER: return "SUPER / USB3 Gen1 (5G)";
    case dai::UsbSpeed::SUPER_PLUS: return "SUPER_PLUS / USB3 Gen2 (10G)";
    default: return "UNKNOWN";
  }
}

const char* sensorTypeName(dai::CameraSensorType type) {
  switch (type) {
    case dai::CameraSensorType::COLOR: return "COLOR";
    case dai::CameraSensorType::MONO: return "MONO";
    case dai::CameraSensorType::TOF: return "TOF";
    case dai::CameraSensorType::THERMAL: return "THERMAL";
    default: return "AUTO";
  }
}

void printCameraFeatures(dai::Device& device) {
  for (const auto& cam : device.getConnectedCameraFeatures()) {
    std::printf("  %s sensor=%s supportedTypes=[", ("CAM_" + std::string(1, static_cast<char>('A' + static_cast<int>(cam.socket)))).c_str(), cam.sensorName.c_str());
    for (const auto type : cam.supportedTypes) std::printf(" %s", sensorTypeName(type));
    std::printf(" ] configs:");
    for (const auto& c : cam.configs) {
      std::printf(" %dx%d@%.0f(%s)", c.width, c.height, c.maxFps, sensorTypeName(c.type));
    }
    std::printf("\n");
  }
}

}  // namespace

int main(int argc, char** argv) {
  const auto args = parseArgs(argc, argv);
  const std::vector<std::pair<std::string, dai::CameraBoardSocket>> all_sockets = {
      {"CAM_A", dai::CameraBoardSocket::CAM_A},
      {"CAM_B", dai::CameraBoardSocket::CAM_B},
      {"CAM_C", dai::CameraBoardSocket::CAM_C},
      {"CAM_D", dai::CameraBoardSocket::CAM_D},
  };
  const bool is_400p = args.res == "400p";

  dai::Pipeline pipeline;
  pipeline.setXLinkChunkSize(args.chunk);
  auto sync = pipeline.create<dai::node::Sync>();
  sync->setSyncThreshold(std::chrono::milliseconds(30));
  auto xout = pipeline.create<dai::node::XLinkOut>();
  xout->setStreamName("sync");
  sync->out.link(xout->input);

  for (int i = 0; i < args.sockets; ++i) {
    const auto& [name, socket] = all_sockets.at(i);
    const auto sync_mode = i == 0 ? dai::CameraControl::FrameSyncMode::OUTPUT : dai::CameraControl::FrameSyncMode::INPUT;
    if (args.mode == "color-nv12") {
      auto cam = pipeline.create<dai::node::ColorCamera>();
      cam->setBoardSocket(socket);
      cam->setResolution(dai::ColorCameraProperties::SensorResolution::THE_800_P);
      if (is_400p) cam->setIspScale(1, 2);
      cam->setVideoSize(is_400p ? 640 : 1280, is_400p ? 400 : 800);
      cam->setFps(args.fps);
      cam->initialControl.setFrameSyncMode(sync_mode);
      cam->video.link(sync->inputs[name]);
    } else {
      auto cam = pipeline.create<dai::node::MonoCamera>();
      cam->setBoardSocket(socket);
      cam->setResolution(is_400p ? dai::MonoCameraProperties::SensorResolution::THE_400_P
                                 : dai::MonoCameraProperties::SensorResolution::THE_800_P);
      cam->setFps(args.fps);
      cam->initialControl.setFrameSyncMode(sync_mode);
      if (args.mode == "mono-mjpeg") {
        auto enc = pipeline.create<dai::node::VideoEncoder>();
        enc->setDefaultProfilePreset(args.fps, dai::VideoEncoderProperties::Profile::MJPEG);
        if (args.quality > 0) enc->setQuality(args.quality);
        if (args.quality < 0) enc->setLossless(true);
        cam->out.link(enc->input);
        enc->bitstream.link(sync->inputs[name]);
      } else {
        cam->out.link(sync->inputs[name]);
      }
    }
  }

  auto imu = pipeline.create<dai::node::IMU>();
  auto ximu = pipeline.create<dai::node::XLinkOut>();
  ximu->setStreamName("imu");
  imu->enableIMUSensor({dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW}, 200);
  imu->setBatchReportThreshold(1);
  imu->setMaxBatchReports(10);
  imu->out.link(ximu->input);

  dai::BoardConfig board_config;
  board_config.gpio[42] = dai::BoardConfig::GPIO(
      dai::BoardConfig::GPIO::INPUT, dai::BoardConfig::GPIO::HIGH, dai::BoardConfig::GPIO::PULL_DOWN);
  pipeline.setBoardConfig(board_config);

  const auto max_speed = args.usb == "super" ? dai::UsbSpeed::SUPER : dai::UsbSpeed::HIGH;
  dai::Device device(pipeline, max_speed);
  std::printf("mode=%s res=%s fps=%d sockets=%d maxUsb=%s\n", args.mode.c_str(), args.res.c_str(), args.fps,
              args.sockets, args.usb.c_str());
  std::printf("negotiated USB speed: %s\n", usbSpeedName(device.getUsbSpeed()));
  printCameraFeatures(device);

  auto q = device.getOutputQueue("sync", 8, false);
  auto qimu = device.getOutputQueue("imu", 50, false);

  // 前 3 秒丢弃（AE 收敛、队列建立），之后按秒统计。
  const auto warmup = std::chrono::seconds(3);
  const auto start = std::chrono::steady_clock::now();
  auto window_start = start + warmup;
  long groups = 0, total_groups = 0, imu_pkts = 0, total_imu = 0;
  double bytes = 0, total_bytes = 0, total_s = 0;
  std::string encoding;
  int64_t last_seq = -1, seq_gaps = 0;

  while (std::chrono::steady_clock::now() - start < warmup + std::chrono::seconds(args.seconds)) {
    while (auto m = qimu->tryGet<dai::IMUData>()) {
      if (std::chrono::steady_clock::now() >= start + warmup) { ++imu_pkts; imu_pkts += m->packets.size() - 1; }
    }
    auto group = q->tryGet<dai::MessageGroup>();
    if (!group) {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }
    const auto now = std::chrono::steady_clock::now();
    if (now < start + warmup) continue;
    if (group->getNumMessages() != static_cast<size_t>(args.sockets)) continue;
    for (const auto& name : group->getMessageNames()) {
      auto f = group->get<dai::ImgFrame>(name);
      bytes += f->getData().size();
      if (name == "CAM_A") {
        if (last_seq >= 0 && f->getSequenceNum() != last_seq + 1) seq_gaps += f->getSequenceNum() - last_seq - 1;
        last_seq = f->getSequenceNum();
        if (encoding.empty()) {
          encoding = std::to_string(f->getWidth()) + "x" + std::to_string(f->getHeight()) + " type=" +
                     std::to_string(static_cast<int>(f->getType())) + " bytes=" + std::to_string(f->getData().size());
          std::printf("frame: %s\n", encoding.c_str());
          if (!args.dump.empty()) {
            std::ofstream(args.dump, std::ios::binary)
                .write(reinterpret_cast<const char*>(f->getData().data()), f->getData().size());
          }
        }
      }
    }
    ++groups;
    if (now - window_start >= std::chrono::seconds(1)) {
      const double dt = std::chrono::duration<double>(now - window_start).count();
      std::printf("  %.1f groups/s  %.2f MB/s  imu %.0f Hz\n", groups / dt, bytes / dt / 1e6, imu_pkts / dt);
      std::fflush(stdout);
      total_groups += groups;
      total_bytes += bytes;
      total_imu += imu_pkts;
      total_s += dt;
      groups = 0;
      bytes = 0;
      imu_pkts = 0;
      window_start = now;
    }
  }
  std::printf("SUMMARY mode=%s usb=%s: %.2f groups/s (target %d), %.2f MB/s, imu %.0f Hz, CAM_A seq gaps=%ld\n",
              args.mode.c_str(), usbSpeedName(device.getUsbSpeed()), total_groups / total_s, args.fps,
              total_bytes / total_s / 1e6, total_imu / total_s, static_cast<long>(seq_gaps));
  return 0;
}
