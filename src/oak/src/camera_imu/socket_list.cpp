#include "camera_imu/socket_list.h"

#include <algorithm>
#include <cctype>
#include <sstream>
#include <stdexcept>

namespace oak_ffc_camera_imu {
namespace {

std::string trim(std::string value) {
  const auto first = value.find_first_not_of(" \t\n\r[]");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = value.find_last_not_of(" \t\n\r[]");
  return value.substr(first, last - first + 1);
}

bool contains(const std::vector<std::string>& items, const std::string& item) {
  return std::find(items.begin(), items.end(), item) != items.end();
}

}  // namespace

const std::vector<std::string>& allSocketNames() {
  static const std::vector<std::string> names = {"CAM_A", "CAM_B", "CAM_C", "CAM_D"};
  return names;
}

void declareSocketListParameter(rclcpp::Node& node, const std::vector<std::string>& defaults) {
  rcl_interfaces::msg::ParameterDescriptor descriptor;
  descriptor.dynamic_typing = true;
  descriptor.description = "camera sockets, e.g. [CAM_A, CAM_B]; ALL = auto-detect connected cameras";
  node.declare_parameter("cam_board_sockets", rclcpp::ParameterValue(defaults), descriptor);
}

std::vector<std::string> parseSocketList(const rclcpp::Parameter& parameter) {
  std::vector<std::string> raw;
  if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING_ARRAY) {
    raw = parameter.as_string_array();
  } else if (parameter.get_type() == rclcpp::ParameterType::PARAMETER_STRING) {
    std::stringstream stream(parameter.as_string());
    std::string item;
    while (std::getline(stream, item, ',')) {
      raw.push_back(item);
    }
  } else {
    throw std::runtime_error("cam_board_sockets must be a string array or comma-separated string");
  }

  std::vector<std::string> sockets;
  for (const auto& item : raw) {
    auto socket = trim(item);
    std::transform(socket.begin(), socket.end(), socket.begin(), [](unsigned char c) { return std::toupper(c); });
    if (socket.empty()) {
      continue;
    }
    if (contains(sockets, socket)) {
      throw std::runtime_error("cam_board_sockets: duplicate socket " + socket);
    }
    sockets.push_back(socket);
  }

  if (contains(sockets, kAllSockets) && sockets.size() != 1) {
    throw std::runtime_error("cam_board_sockets: ALL cannot be combined with explicit sockets");
  }
  return sockets;
}

bool isAutoDetect(const std::vector<std::string>& sockets) {
  return sockets.size() == 1 && sockets.front() == kAllSockets;
}

std::vector<std::string> expandSockets(const std::vector<std::string>& sockets) {
  return isAutoDetect(sockets) ? allSocketNames() : sockets;
}

SocketSelection selectSockets(const std::vector<std::string>& requested, const std::vector<std::string>& detected) {
  SocketSelection selection;
  for (const auto& socket : expandSockets(requested)) {
    if (contains(detected, socket)) {
      selection.selected.push_back(socket);
    } else if (!isAutoDetect(requested)) {
      selection.missing.push_back(socket);
    }
  }
  return selection;
}

}  // namespace oak_ffc_camera_imu
