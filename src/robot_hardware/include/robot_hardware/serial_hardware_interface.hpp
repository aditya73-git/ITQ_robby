#pragma once

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_interface_params.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace robot_hardware
{

class SerialHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SerialHardwareInterface)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  int openSerial(const std::string & port, int baud_rate);
  void closeSerial();

  std::string serial_port_;
  int baud_rate_{115200};
  double max_wheel_angular_vel_{13.3};  // rad/s → corresponds to ~1 m/s at r=0.075m

  int serial_fd_{-1};

  // Indexed in joint order from URDF: front_left, front_right, rear_left, rear_right
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  // Indices of left and right joints after on_init parses joint names
  std::vector<size_t> left_indices_;
  std::vector<size_t> right_indices_;
};

}  // namespace robot_hardware
