#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "camera_imu/pipeline_builder.h"

namespace oak_ffc_camera_imu {
namespace {

DriverConfig configWithSockets() {
  DriverConfig config;
  config.cam_board_sockets = {"CAM_C", "CAM_D"};
  return config;
}

bool hasImuNode(const dai::Pipeline& pipeline) {
  for (const auto& node : pipeline.getAllNodes()) {
    if (std::dynamic_pointer_cast<const dai::node::IMU>(node)) {
      return true;
    }
  }
  return false;
}

bool hasXLinkOutStream(const dai::Pipeline& pipeline, const std::string& stream) {
  for (const auto& node : pipeline.getAllNodes()) {
    const auto x_out = std::dynamic_pointer_cast<const dai::node::XLinkOut>(node);
    if (x_out && x_out->getStreamName() == stream) {
      return true;
    }
  }
  return false;
}

std::vector<float> colorCameraFps(const dai::Pipeline& pipeline) {
  std::vector<float> fps;
  for (const auto& node : pipeline.getAllNodes()) {
    if (const auto camera = std::dynamic_pointer_cast<const dai::node::ColorCamera>(node)) {
      fps.push_back(camera->getFps());
    }
  }
  return fps;
}

TEST(PipelineBuilder, Ov9782SensorRunsAtTargetFps) {
  // OV9782 由首路 sensor 输出 FSIN，帧率就是 sensor 自己的 fps。
  auto config = configWithSockets();
  config.camera_name = "ov9782";
  config.fps = 30;
  EXPECT_EQ(colorCameraFps(createPipeline(config).pipeline), (std::vector<float>{30.0F, 30.0F}));
}

TEST(PipelineBuilder, ScriptTriggeredSensorHasFrameTimeHeadroom) {
  // AR0234 由脚本按 fps 打 FSIN；sensor 帧时长要比触发周期短，抖动早到的脉冲才不会被丢。
  auto config = configWithSockets();
  config.camera_name = "ar0234";
  config.fps = 30;
  EXPECT_EQ(colorCameraFps(createPipeline(config).pipeline), (std::vector<float>{60.0F, 60.0F}));
}

TEST(PipelineBuilder, IncludesImuByDefault) {
  const auto bundle = createPipeline(configWithSockets());
  EXPECT_TRUE(hasImuNode(bundle.pipeline));
  EXPECT_TRUE(hasXLinkOutStream(bundle.pipeline, "imu"));
}

TEST(PipelineBuilder, NoImuLeavesImuOut) {
  auto config = configWithSockets();
  config.noimu = true;
  const auto bundle = createPipeline(config);
  EXPECT_FALSE(hasImuNode(bundle.pipeline));
  EXPECT_FALSE(hasXLinkOutStream(bundle.pipeline, "imu"));
  // 图像通路不受影响。
  EXPECT_TRUE(hasXLinkOutStream(bundle.pipeline, "sync"));
}

}  // namespace
}  // namespace oak_ffc_camera_imu
