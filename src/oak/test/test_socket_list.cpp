#include <gtest/gtest.h>

#include "camera_imu/socket_list.h"

using oak_ffc_camera_imu::expandSockets;
using oak_ffc_camera_imu::isAutoDetect;
using oak_ffc_camera_imu::parseSocketList;
using oak_ffc_camera_imu::selectSockets;
using Sockets = std::vector<std::string>;

TEST(SocketList, ParsesLaunchStyleString) {
  EXPECT_EQ(parseSocketList(rclcpp::Parameter("s", "[CAM_A,CAM_B, CAM_C]")), (Sockets{"CAM_A", "CAM_B", "CAM_C"}));
}

TEST(SocketList, ParsesStringArray) {
  EXPECT_EQ(parseSocketList(rclcpp::Parameter("s", Sockets{"CAM_C", " CAM_D "})), (Sockets{"CAM_C", "CAM_D"}));
}

TEST(SocketList, AllIsCaseInsensitiveInEitherForm) {
  EXPECT_TRUE(isAutoDetect(parseSocketList(rclcpp::Parameter("s", "all"))));
  EXPECT_TRUE(isAutoDetect(parseSocketList(rclcpp::Parameter("s", "[ALL]"))));
  EXPECT_TRUE(isAutoDetect(parseSocketList(rclcpp::Parameter("s", Sockets{"ALL"}))));
}

TEST(SocketList, RejectsAllMixedWithSockets) {
  EXPECT_THROW(parseSocketList(rclcpp::Parameter("s", "[ALL, CAM_A]")), std::runtime_error);
}

TEST(SocketList, RejectsDuplicateSockets) {
  EXPECT_THROW(parseSocketList(rclcpp::Parameter("s", "[CAM_A, cam_a]")), std::runtime_error);
}

TEST(SocketList, ExpandAllGivesEverySocket) {
  EXPECT_EQ(expandSockets({"ALL"}), (Sockets{"CAM_A", "CAM_B", "CAM_C", "CAM_D"}));
  EXPECT_EQ(expandSockets({"CAM_B"}), (Sockets{"CAM_B"}));
}

TEST(SocketList, AutoDetectPicksDetectedWithoutReportingMissing) {
  const auto selection = selectSockets({"ALL"}, {"CAM_B", "CAM_A"});
  EXPECT_EQ(selection.selected, (Sockets{"CAM_A", "CAM_B"}));
  EXPECT_TRUE(selection.missing.empty());
}

TEST(SocketList, ExplicitListDropsAndReportsMissing) {
  const auto selection = selectSockets({"CAM_D", "CAM_A", "CAM_C"}, {"CAM_A", "CAM_B"});
  EXPECT_EQ(selection.selected, (Sockets{"CAM_A"}));
  EXPECT_EQ(selection.missing, (Sockets{"CAM_D", "CAM_C"}));
}

TEST(SocketList, NothingDetectedSelectsNothing) {
  EXPECT_TRUE(selectSockets({"ALL"}, {}).selected.empty());
}
