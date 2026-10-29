#include "camera_imu/camera_probe.h"

#include <map>
#include <set>
#include <stdexcept>
#include <thread>

#include "camera_imu/pipeline_builder.h"

namespace oak_ffc_camera_imu {
namespace {

std::string probeStream(const std::string& socket) { return "probe_" + socket; }

}  // namespace

std::vector<std::string> reportedSockets(dai::Device& device) {
  std::vector<std::string> names;
  for (const auto socket : device.getConnectedCameras()) {
    for (const auto& option : cameraSocketOptions()) {
      if (option.second.socket == socket) {
        names.push_back(option.first);
      }
    }
  }
  return names;
}

std::vector<std::string> probeStreamingSockets(
    dai::Device& device,
    const DriverConfig& config,
    const std::vector<std::string>& candidates,
    std::chrono::milliseconds timeout) {
  if (candidates.empty()) {
    return {};
  }

  // 与正式 pipeline 同样的 sensor 模式，但不打帧同步（默认 FrameSyncMode OFF，自由运行），
  // 也不做缩放外的任何处理，只看各路有没有帧。
  const auto resolution = resolveResolution(config);
  dai::Pipeline pipeline;
  pipeline.setBoardConfig(deviceConfig(config).board);
  for (const auto& socket_name : candidates) {
    auto x_out = pipeline.create<dai::node::XLinkOut>();
    x_out->setStreamName(probeStream(socket_name));
    if (config.camera_type == CameraType::Color) {
      auto camera = pipeline.create<dai::node::ColorCamera>();
      camera->setBoardSocket(cameraSocketOptions().at(socket_name).socket);
      camera->setResolution(resolution.color_resolution);
      if (resolution.isp_scale_num != resolution.isp_scale_den) {
        camera->setIspScale(resolution.isp_scale_num, resolution.isp_scale_den);
      }
      camera->setFps(config.fps);
      camera->isp.link(x_out->input);
    } else {
      auto camera = pipeline.create<dai::node::MonoCamera>();
      camera->setBoardSocket(cameraSocketOptions().at(socket_name).socket);
      camera->setResolution(resolution.mono_resolution);
      camera->setFps(config.fps);
      camera->out.link(x_out->input);
    }
  }
  if (!device.startPipeline(pipeline)) {
    throw std::runtime_error("failed to start camera probe pipeline");
  }

  std::map<std::string, std::shared_ptr<dai::DataOutputQueue>> queues;
  for (const auto& socket_name : candidates) {
    queues[socket_name] = device.getOutputQueue(probeStream(socket_name), 2, false);
  }

  std::set<std::string> streaming;
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (streaming.size() < candidates.size() && std::chrono::steady_clock::now() < deadline) {
    for (const auto& queue : queues) {
      if (queue.second->tryGet<dai::ImgFrame>()) {
        streaming.insert(queue.first);
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  std::vector<std::string> result;
  for (const auto& socket_name : candidates) {
    if (streaming.count(socket_name) > 0) {
      result.push_back(socket_name);
    }
  }
  return result;
}

}  // namespace oak_ffc_camera_imu
