#include "robot_hardware/serial_hardware_interface.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace robot_hardware
{

hardware_interface::CallbackReturn SerialHardwareInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Read parameters from URDF <param> tags
  serial_port_ = info_.hardware_parameters.count("serial_port")
    ? info_.hardware_parameters.at("serial_port")
    : "/dev/ttyACM0";

  if (info_.hardware_parameters.count("baud_rate")) {
    baud_rate_ = std::stoi(info_.hardware_parameters.at("baud_rate"));
  }

  if (info_.hardware_parameters.count("max_wheel_angular_vel")) {
    max_wheel_angular_vel_ = std::stod(info_.hardware_parameters.at("max_wheel_angular_vel"));
  }

  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);

  // Classify joints as left or right by name
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    const std::string & name = info_.joints[i].name;
    if (name.find("left") != std::string::npos) {
      left_indices_.push_back(i);
    } else if (name.find("right") != std::string::npos) {
      right_indices_.push_back(i);
    } else {
      RCLCPP_WARN(rclcpp::get_logger("SerialHardwareInterface"),
        "Joint '%s' is neither left nor right — ignored in serial output", name.c_str());
    }
  }

  if (left_indices_.empty() || right_indices_.empty()) {
    RCLCPP_ERROR(rclcpp::get_logger("SerialHardwareInterface"),
      "Could not find left/right joints. Check joint names in URDF.");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SerialHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SerialHardwareInterface"),
    "Configured: port=%s baud=%d max_wheel_vel=%.1f rad/s",
    serial_port_.c_str(), baud_rate_, max_wheel_angular_vel_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SerialHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  serial_fd_ = openSerial(serial_port_, baud_rate_);
  if (serial_fd_ < 0) {
    // Non-fatal: ESP32 may not be connected yet; write() will keep retrying
    RCLCPP_WARN(rclcpp::get_logger("SerialHardwareInterface"),
      "Serial port '%s' not available at startup — will retry on write.", serial_port_.c_str());
  } else {
    RCLCPP_INFO(rclcpp::get_logger("SerialHardwareInterface"),
      "Serial port '%s' opened.", serial_port_.c_str());
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SerialHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Send stop command before closing
  if (serial_fd_ >= 0) {
    const char * stop = "0,0\n";
    ::write(serial_fd_, stop, 4);
  }
  closeSerial();
  RCLCPP_INFO(rclcpp::get_logger("SerialHardwareInterface"), "Serial port closed.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SerialHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    state_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SerialHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    command_interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_commands_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type SerialHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // No encoders — integrate commanded velocity as estimated position
  for (size_t i = 0; i < hw_commands_.size(); ++i) {
    hw_velocities_[i] = hw_commands_[i];
    hw_positions_[i] += hw_velocities_[i] * period.seconds();
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SerialHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (serial_fd_ < 0) {
    return hardware_interface::return_type::OK;
  }

  // Average left and right wheel commands (rad/s)
  double left_vel = 0.0;
  for (size_t idx : left_indices_) {
    left_vel += hw_commands_[idx];
  }
  left_vel /= static_cast<double>(left_indices_.size());

  double right_vel = 0.0;
  for (size_t idx : right_indices_) {
    right_vel += hw_commands_[idx];
  }
  right_vel /= static_cast<double>(right_indices_.size());

  // Convert to fwd/steer in -255..255 (matches ESP32 protocol from Pi)
  // left = fwd + steer,  right = fwd - steer
  // => fwd = (left + right)/2,  steer = (left - right)/2
  const double scale = 255.0 / max_wheel_angular_vel_;
  int fwd   = static_cast<int>(std::round((left_vel + right_vel) / 2.0 * scale));
  int steer = static_cast<int>(std::round((left_vel - right_vel) / 2.0 * scale));

  fwd   = std::clamp(fwd,   -255, 255);
  steer = std::clamp(steer, -255, 255);

  char msg[16];
  int len = std::snprintf(msg, sizeof(msg), "%d,%d\n", fwd, steer);

  if (::write(serial_fd_, msg, len) < 0) {
    RCLCPP_WARN(rclcpp::get_logger("SerialHardwareInterface"), "Serial write failed.");
    closeSerial();
    // Attempt reconnect on next write cycle
    serial_fd_ = openSerial(serial_port_, baud_rate_);
  }

  return hardware_interface::return_type::OK;
}

int SerialHardwareInterface::openSerial(const std::string & port, int baud_rate)
{
  int fd = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
  if (fd < 0) {
    return -1;
  }

  struct termios tty{};
  tcgetattr(fd, &tty);

  speed_t speed = B115200;
  switch (baud_rate) {
    case 9600:   speed = B9600;   break;
    case 57600:  speed = B57600;  break;
    case 115200: speed = B115200; break;
    default:     speed = B115200; break;
  }

  cfsetospeed(&tty, speed);
  cfsetispeed(&tty, speed);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_iflag &= ~(IXON | IXOFF | IXANY);
  tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  tty.c_oflag &= ~OPOST;
  tcsetattr(fd, TCSANOW, &tty);

  return fd;
}

void SerialHardwareInterface::closeSerial()
{
  if (serial_fd_ >= 0) {
    ::close(serial_fd_);
    serial_fd_ = -1;
  }
}

}  // namespace robot_hardware

PLUGINLIB_EXPORT_CLASS(
  robot_hardware::SerialHardwareInterface,
  hardware_interface::SystemInterface)
