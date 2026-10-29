#include <gtest/gtest.h>

#include "camera_imu/fps_display.h"

using oak_ffc_camera_imu::computeFps;
using oak_ffc_camera_imu::computeRate;
using oak_ffc_camera_imu::formatFpsLine;
using oak_ffc_camera_imu::FpsBySocket;
using oak_ffc_camera_imu::FrameCounts;

TEST(FpsDisplay, ComputesRateFromCountDelta) {
  const auto fps = computeFps({{"CAM_A", 100}, {"CAM_B", 90}}, {{"CAM_A", 130}, {"CAM_B", 105}}, 1.0);
  ASSERT_EQ(fps.size(), 2u);
  EXPECT_DOUBLE_EQ(*fps.at("CAM_A"), 30.0);
  EXPECT_DOUBLE_EQ(*fps.at("CAM_B"), 15.0);
}

TEST(FpsDisplay, ScalesByElapsedTime) {
  const auto fps = computeFps({{"CAM_A", 30}}, {{"CAM_A", 90}}, 2.0);
  EXPECT_DOUBLE_EQ(*fps.at("CAM_A"), 30.0);
}

TEST(FpsDisplay, UnknownWhenSocketIsNewOrCountWentBack) {
  // 刚连上没有上一次采样；重连后计数清零会比上次小，这两种都算不出这一段的帧率。
  const auto fps = computeFps({{"CAM_A", 500}}, {{"CAM_A", 10}, {"CAM_B", 10}}, 1.0);
  EXPECT_FALSE(fps.at("CAM_A").has_value());
  EXPECT_FALSE(fps.at("CAM_B").has_value());
}

TEST(FpsDisplay, OnlyReportsCurrentSockets) {
  // 断连后被剔除的插座不再显示。
  const auto fps = computeFps({{"CAM_A", 30}, {"CAM_C", 30}}, {{"CAM_A", 60}}, 1.0);
  EXPECT_EQ(fps.size(), 1u);
  EXPECT_EQ(fps.count("CAM_C"), 0u);
}

TEST(FpsDisplay, UnknownWhenNoTimeElapsed) {
  EXPECT_FALSE(computeFps({{"CAM_A", 30}}, {{"CAM_A", 60}}, 0.0).at("CAM_A").has_value());
}

TEST(FpsDisplay, ImuRateFromPacketCount) {
  EXPECT_DOUBLE_EQ(*computeRate(1000, 1400, 2.0), 200.0);
  EXPECT_FALSE(computeRate(std::nullopt, 200, 1.0).has_value());
  EXPECT_FALSE(computeRate(1000, 10, 1.0).has_value());
}

TEST(FpsDisplay, UnknownUntilFirstFullInterval) {
  // 连上后第一段区间里首帧来得晚，算出来的 27fps 并不是掉帧，不能标红误报。
  EXPECT_FALSE(computeRate(0, 27, 1.0).has_value());
  EXPECT_DOUBLE_EQ(*computeRate(27, 57, 1.0), 30.0);
}

TEST(FpsDisplay, FormatsCamerasAndImuWithTargets) {
  const FpsBySocket fps = {{"CAM_A", 30.0}, {"CAM_B", 29.94}, {"CAM_C", std::nullopt}};
  EXPECT_EQ(
      formatFpsLine("oak", fps, 30, 199.6, 200, false),
      "[oak fps] CAM_A 30.0  CAM_B 29.9  CAM_C  --  /30  IMU 200 /200");
}

TEST(FpsDisplay, FourCamerasAndImuFitIn80Columns) {
  // 超过终端宽度会折行，\r 只能回到折行后的行首，刷新就乱了。
  const FpsBySocket fps = {{"CAM_A", 30.0}, {"CAM_B", 30.0}, {"CAM_C", 30.0}, {"CAM_D", 30.0}};
  EXPECT_LE(formatFpsLine("oak", fps, 30, 200.0, 200, false).size(), 80u);
}

TEST(FpsDisplay, FormatsUnknownImu) {
  EXPECT_EQ(
      formatFpsLine("oak", {{"CAM_A", 30.0}}, 30, std::nullopt, 200, false), "[oak fps] CAM_A 30.0 /30  IMU  -- /200");
}

TEST(FpsDisplay, FormatsWaitingWhenDisconnected) {
  EXPECT_EQ(formatFpsLine("oak", {}, 30, std::nullopt, 200, false), "[oak fps] waiting for device...");
}

TEST(FpsDisplay, HighlightsOnlyRatesBelowTarget) {
  const auto line = formatFpsLine("oak", {{"CAM_A", 30.0}, {"CAM_B", 24.0}}, 30, 150.0, 200, true);
  EXPECT_NE(line.find("\033[31m24.0\033[0m"), std::string::npos);
  EXPECT_EQ(line.find("\033[31m30.0"), std::string::npos);
  EXPECT_NE(line.find("\033[31m150\033[0m"), std::string::npos);
}

TEST(FpsDisplay, DoesNotHighlightImuAtTarget) {
  EXPECT_EQ(formatFpsLine("oak", {{"CAM_A", 30.0}}, 30, 200.0, 200, true).find("\033[31m"), std::string::npos);
}
