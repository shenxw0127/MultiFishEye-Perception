#include "camera_imu/pipeline_builder.h"

#include <chrono>
#include <stdexcept>

#include "camera_imu/camera_controls.h"

namespace oak_ffc_camera_imu {
namespace {

std::string fsyncScript(int fps) {
  return std::string(R"(# coding=utf-8
import time
import GPIO

fps = )") + std::to_string(fps) + std::string(R"(
calib = Device.readCalibration2().getEepromData()
boardRev = calib.boardRev

revision = -1
if len(boardRev) >= 2 and boardRev[0] == 'R':
    revision = int(boardRev[1])

GPIO_FSIN_2LANE = 41
GPIO_FSIN_4LANE = 40
GPIO_FSIN_MODE_SELECT = 6

if revision >= 6:
    GPIO_FSIN_2LANE = 41
    GPIO_FSIN_4LANE = 42
    GPIO_FSIN_MODE_SELECT = 38

GPIO.setup(GPIO_FSIN_2LANE, GPIO.OUT)
GPIO.write(GPIO_FSIN_2LANE, 0)
GPIO.setup(GPIO_FSIN_4LANE, GPIO.IN)
GPIO.setup(GPIO_FSIN_MODE_SELECT, GPIO.OUT)
GPIO.write(GPIO_FSIN_MODE_SELECT, 1)

period = 1 / fps
active = 0.001
overhead = 0.003

while True:
    GPIO.write(GPIO_FSIN_2LANE, 1)
    time.sleep(active)
    GPIO.write(GPIO_FSIN_2LANE, 0)
    time.sleep(period - active - overhead)
)");
}

bool usesSensorFsyncMaster(const DriverConfig& config) { return config.camera_name == "ov9782"; }

// OV9782：首路 sensor 作 FSIN 主机输出脉冲，其余跟随。
// AR0234 等其它 sensor：由 LEON_CSS 上的 fsyncScript 用 GPIO 打 FSIN 脉冲，所有 sensor
// 都必须是 INPUT，否则首路 sensor 会和脚本同时驱动 FSIN 线（与厂商参考实现一致）。
dai::CameraControl::FrameSyncMode frameSyncMode(const DriverConfig& config, const std::string& board_socket_name) {
  if (usesSensorFsyncMaster(config) && board_socket_name == config.cam_board_sockets.front()) {
    return dai::CameraControl::FrameSyncMode::OUTPUT;
  }
  return dai::CameraControl::FrameSyncMode::INPUT;
}

}  // namespace

PipelineBundle createPipeline(const DriverConfig& config) {
  if (isAutoDetect(config.cam_board_sockets)) {
    throw std::logic_error("createPipeline needs resolved camera sockets, got ALL");
  }
  PipelineBundle bundle;
  bundle.pipeline.setXLinkChunkSize(0);

  const auto resolution = resolveResolution(config);
  bundle.width = resolution.width;
  bundle.height = resolution.height;

  auto sync = bundle.pipeline.create<dai::node::Sync>();
  sync->setSyncThreshold(std::chrono::milliseconds(config.sync_threshold_ms));

  auto x_sync_out = bundle.pipeline.create<dai::node::XLinkOut>();
  x_sync_out->setStreamName("sync");
  sync->out.link(x_sync_out->input);

  for (const auto& board_socket_name : config.cam_board_sockets) {
    const auto socket_config = cameraSocketOptions().at(board_socket_name);
    auto video_encoder = bundle.pipeline.create<dai::node::VideoEncoder>();
    auto control_in = bundle.pipeline.create<dai::node::XLinkIn>();
    const auto control_stream = "control_" + board_socket_name;
    control_in->setStreamName(control_stream);

    const bool encode = deviceEncodesMjpeg(config);
    if (encode) {
      video_encoder->setDefaultProfilePreset(config.fps, dai::VideoEncoderProperties::Profile::MJPEG);
      video_encoder->setQuality(config.jpeg_quality);
      video_encoder->bitstream.link(sync->inputs[board_socket_name]);
    }

    if (config.camera_type == CameraType::Color) {
      auto camera = bundle.pipeline.create<dai::node::ColorCamera>();
      camera->setBoardSocket(socket_config.socket);
      camera->setResolution(resolution.color_resolution);
      if (resolution.isp_scale_num != resolution.isp_scale_den) {
        // ISP 硬件缩放，VPU 上完成，主机端零开销，且按像素缩放不裁视场。
        camera->setIspScale(resolution.isp_scale_num, resolution.isp_scale_den);
      }
      camera->setFps(config.fps);
      camera->inputControl.setBlocking(false);
      camera->inputControl.setQueueSize(4);
      camera->inputControl.setWaitForMessage(false);
      control_in->out.link(camera->inputControl);

      if (encode) {
        // compressed 走 video 流，显式钉死输出尺寸，保证和 bundle.width/height 一致。
        camera->setVideoSize(resolution.width, resolution.height);
        camera->video.link(video_encoder->input);
      } else {
        // raw 模式：preview 输出交错 BGR (BGR888i)，避免 ISP 的 YUV420p 平面歧义。
        // 下游 cv_bridge::toCvShare 可真正零拷贝，且 foxglove/rosbag 原生支持 bgr8。
        // 颜色空间转换由 OAK VPU 硬件完成，主机端零 CPU 开销。
        camera->setColorOrder(dai::ColorCameraProperties::ColorOrder::BGR);
        camera->setInterleaved(true);
        camera->setPreviewSize(resolution.width, resolution.height);
        camera->setPreviewKeepAspectRatio(false);  // 铺满，1:1 匹配标定分辨率
        camera->preview.link(sync->inputs[board_socket_name]);
      }

      camera->initialControl.setFrameSyncMode(frameSyncMode(config, board_socket_name));
      applyCameraControls(camera->initialControl, config);
    } else {
      auto camera = bundle.pipeline.create<dai::node::MonoCamera>();
      camera->setBoardSocket(socket_config.socket);
      camera->setResolution(resolution.mono_resolution);
      camera->setFps(config.fps);
      camera->inputControl.setBlocking(false);
      camera->inputControl.setQueueSize(4);
      camera->inputControl.setWaitForMessage(false);
      control_in->out.link(camera->inputControl);

      // MonoCamera 由厂商固件按 MONO 模式驱动 sensor，直接出 GRAY8（OV9782 这类彩色
      // sensor 也一样由 ISP 出亮度），不经过彩色图，主机端也无需再转灰度。
      if (encode) {
        camera->out.link(video_encoder->input);
      } else {
        camera->out.link(sync->inputs[board_socket_name]);
      }

      camera->initialControl.setFrameSyncMode(frameSyncMode(config, board_socket_name));
      applyCameraControls(camera->initialControl, config);
    }

    bundle.control_streams[board_socket_name] = control_stream;
  }

  bundle.image_topics = imageTopics(config);

  auto imu = bundle.pipeline.create<dai::node::IMU>();
  auto x_imu_out = bundle.pipeline.create<dai::node::XLinkOut>();
  x_imu_out->setStreamName("imu");
  imu->enableIMUSensor(
      {dai::IMUSensor::ROTATION_VECTOR, dai::IMUSensor::ACCELEROMETER_RAW, dai::IMUSensor::GYROSCOPE_RAW},
      config.imu_hz);
  imu->setBatchReportThreshold(1);
  imu->setMaxBatchReports(10);
  imu->out.link(x_imu_out->input);

  bundle.pipeline.setBoardConfig(deviceConfig(config).board);
  if (!usesSensorFsyncMaster(config)) {
    auto script = bundle.pipeline.create<dai::node::Script>();
    script->setProcessor(dai::ProcessorType::LEON_CSS);
    script->setScript(fsyncScript(config.fps));
  }

  return bundle;
}

dai::Device::Config deviceConfig(const DriverConfig& config) {
  dai::Device::Config device_config;
  if (usesSensorFsyncMaster(config)) {
    device_config.board.gpio[42] =
        dai::BoardConfig::GPIO(dai::BoardConfig::GPIO::INPUT, dai::BoardConfig::GPIO::HIGH, dai::BoardConfig::GPIO::PULL_DOWN);
  }
  return device_config;
}

std::map<std::string, std::string> imageTopics(const DriverConfig& config) {
  std::map<std::string, std::string> topics;
  for (const auto& board_socket_name : expandSockets(config.cam_board_sockets)) {
    topics[board_socket_name] =
        cameraSocketOptions().at(board_socket_name).image_topic + (config.compressed ? "/compressed" : "");
  }
  return topics;
}

}  // namespace oak_ffc_camera_imu
