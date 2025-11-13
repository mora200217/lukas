#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>
#include <set>
#include <string>
#include <vector>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <angles/angles.h>
#include <rclcpp/executors.hpp>
#include <lukas_control/topic_based_system.hpp>

namespace
{
void sumRotationFromMinus2PiTo2Pi(const double current_wrapped_rad, double& total_rotation)
{
  double delta = 0;
  angles::shortest_angular_distance_with_large_limits(total_rotation, current_wrapped_rad, 2 * M_PI, -2 * M_PI, delta);
  total_rotation += delta;
}
}  // namespace

namespace lukas_control
{

static constexpr std::size_t POSITION_INTERFACE_INDEX = 0;
static constexpr std::size_t VELOCITY_INTERFACE_INDEX = 1;
static constexpr std::size_t EFFORT_INTERFACE_INDEX = 2;

CallbackReturn TopicBasedSystem::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
  {
    return CallbackReturn::ERROR;
  }

  joint_commands_.resize(3);
  joint_states_.resize(3);
  for (auto i = 0u; i < 3; i++)
  {
    joint_commands_[i].resize(info_.joints.size(), 0.0);
    joint_states_[i].resize(info_.joints.size(), 0.0);
  }

  // === UART CONFIG ===
  std::string device = "/dev/tty.usbserial-0001";
  if (auto it = info_.hardware_parameters.find("uart_device"); it != info_.hardware_parameters.end())
  {
    device = it->second;
  }

  uart_fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY);
  if (uart_fd_ < 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("TopicBasedSystem"), "Failed to open UART device: %s", device.c_str());
    return CallbackReturn::ERROR;
  }

  struct termios tty{};
  if (tcgetattr(uart_fd_, &tty) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("TopicBasedSystem"), "Failed to get UART attributes");
    return CallbackReturn::ERROR;
  }

  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_iflag = 0;
  tty.c_oflag = 0;
  tty.c_lflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 10;

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);

  if (tcsetattr(uart_fd_, TCSANOW, &tty) != 0)
  {
    RCLCPP_ERROR(rclcpp::get_logger("TopicBasedSystem"), "Failed to set UART attributes");
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("TopicBasedSystem"), "UART connected on %s", device.c_str());
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> TopicBasedSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    const auto& joint = info_.joints[i];
    for (const auto& interface : joint.state_interfaces)
    {
      if (!getInterface(joint.name, interface.name, i, joint_states_, state_interfaces))
      {
        throw std::runtime_error("Interface not found in standard list.");
      }
    }
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> TopicBasedSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (auto i = 0u; i < info_.joints.size(); i++)
  {
    const auto& joint = info_.joints[i];
    for (const auto& interface : joint.command_interfaces)
    {
      if (!getInterface(joint.name, interface.name, i, joint_commands_, command_interfaces))
      {
        throw std::runtime_error("Interface not found in standard list.");
      }
    }
  }
  return command_interfaces;
}

hardware_interface::return_type TopicBasedSystem::read(const rclcpp::Time&, const rclcpp::Duration&)
{
  char buffer[256];
  int n = ::read(uart_fd_, buffer, sizeof(buffer) - 1);
  if (n > 0)
  {
    buffer[n] = '\0';
    double pos1, pos2;
    if (sscanf(buffer, "%lf,%lf", &pos1, &pos2) == 2)
    {
      joint_states_[POSITION_INTERFACE_INDEX][0] = pos1;
      joint_states_[POSITION_INTERFACE_INDEX][1] = pos2;
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type TopicBasedSystem::write(const rclcpp::Time&, const rclcpp::Duration&)
{
  char cmd[128];
  snprintf(cmd, sizeof(cmd), "%.3f,%.3f\n",
           joint_commands_[POSITION_INTERFACE_INDEX][0],
           joint_commands_[POSITION_INTERFACE_INDEX][1]);
  ::write(uart_fd_, cmd, strlen(cmd));
  return hardware_interface::return_type::OK;
}

template <typename HandleType>
bool TopicBasedSystem::getInterface(
  const std::string& name, const std::string& interface_name,
  const size_t vector_index, std::vector<std::vector<double>>& values,
  std::vector<HandleType>& interfaces)
{
  auto it = std::find(standard_interfaces_.begin(), standard_interfaces_.end(), interface_name);
  if (it != standard_interfaces_.end())
  {
    auto j = static_cast<std::size_t>(std::distance(standard_interfaces_.begin(), it));
    interfaces.emplace_back(name, *it, &values[j][vector_index]);
    return true;
  }
  return false;
}

}  // namespace lukas_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(lukas_control::TopicBasedSystem, hardware_interface::SystemInterface)
