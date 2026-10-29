#include <gtest/gtest.h>

#include <memory>

#include "camera_imu/isp_meta.h"
#include "depthai-shared/datatype/RawImgFrame.hpp"

namespace oak_ffc_camera_imu {
namespace {

std::shared_ptr<dai::ImgFrame> makeFrame(int32_t exposure_us, int32_t iso, int32_t wb_kelvin, int32_t lens, int64_t seq) {
  auto raw = std::make_shared<dai::RawImgFrame>();
  raw->cam.exposureTimeUs = exposure_us;
  raw->cam.sensitivityIso = iso;
  raw->cam.wbColorTemp = wb_kelvin;
  raw->cam.lensPosition = lens;
  raw->sequenceNum = seq;
  return std::make_shared<dai::ImgFrame>(raw);
}

builtin_interfaces::msg::Time stampOf(int32_t sec, uint32_t nsec) {
  builtin_interfaces::msg::Time stamp;
  stamp.sec = sec;
  stamp.nanosec = nsec;
  return stamp;
}

TEST(IspMeta, CopiesFirmwareCameraSettings) {
  const auto frame = makeFrame(2850, 640, 4300, 130, 1234);
  const auto meta = toIspMeta("CAM_A", *frame, stampOf(17, 500));

  EXPECT_EQ(meta.camera, "CAM_A");
  EXPECT_EQ(meta.stamp.sec, 17);
  EXPECT_EQ(meta.stamp.nanosec, 500u);
  EXPECT_EQ(meta.sequence_num, 1234);
  EXPECT_EQ(meta.exposure_time_us, 2850u);
  EXPECT_EQ(meta.iso, 640);
  EXPECT_EQ(meta.color_temperature_k, 4300);
  EXPECT_EQ(meta.lens_position, 130);
  EXPECT_TRUE(hasIspMeta(meta));
}

TEST(IspMeta, MonoCameraHasNoWhiteBalance) {
  const auto frame = makeFrame(3000, 800, 0, -1, 7);
  const auto meta = toIspMeta("CAM_B", *frame, stampOf(0, 0));
  EXPECT_EQ(meta.color_temperature_k, 0);
  EXPECT_EQ(meta.lens_position, -1);
  EXPECT_TRUE(hasIspMeta(meta));
}

TEST(IspMeta, MissingMetadataIsDetectedAndNeverNegative) {
  // 固件没填元数据时 CameraSettings 全 0；负曝光不可能出现，但 uint32 字段不能被它绕成大数。
  const auto zero = toIspMeta("CAM_C", *makeFrame(0, 0, 0, 0, 0), stampOf(0, 0));
  EXPECT_EQ(zero.exposure_time_us, 0u);
  EXPECT_FALSE(hasIspMeta(zero));

  const auto negative = toIspMeta("CAM_D", *makeFrame(-5, 0, 0, 0, 0), stampOf(0, 0));
  EXPECT_EQ(negative.exposure_time_us, 0u);
  EXPECT_FALSE(hasIspMeta(negative));
}

}  // namespace
}  // namespace oak_ffc_camera_imu
