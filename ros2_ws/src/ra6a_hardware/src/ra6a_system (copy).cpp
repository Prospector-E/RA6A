#include "ra6a_hardware/ra6a_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>

namespace ra6a_hardware
{

hardware_interface::CallbackReturn Ra6aSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_mode_ = info_.hardware_parameters["mode"];

  size_t num_joints = info_.joints.size();

  positions_.resize(num_joints, 0.0);
  velocities_.resize(num_joints, 0.0);

  cmd_positions_.resize(num_joints, 0.0);
  cmd_velocities_.resize(num_joints, 0.0);

  RCLCPP_INFO(rclcpp::get_logger("Ra6aSystem"),
              "RA6A Hardware Initialized with %ld joints",
              num_joints);

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Ra6aSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < positions_.size(); ++i)
  {
    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &positions_[i]);

    state_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &velocities_[i]);
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
Ra6aSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < cmd_positions_.size(); ++i)
  {
    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &cmd_positions_[i]);

    command_interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_VELOCITY,
      &cmd_velocities_[i]);
  }

  return command_interfaces;
}

hardware_interface::return_type Ra6aSystem::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  double dt = period.seconds();

  if (hw_mode_ == "fake")
  {
    for (size_t i = 0; i < positions_.size(); ++i)
    {
      double error = cmd_positions_[i] - positions_[i];

      velocities_[i] = error * 5.0;   // simple proportional simulation
      positions_[i] += velocities_[i] * dt;
    }
  }
  else
  {
    // Here you read real position + velocity from STM32
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Ra6aSystem::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  RCLCPP_INFO(rclcpp::get_logger("Ra6aSystem"),
              "---- WRITE ----");

  for (size_t i = 0; i < cmd_positions_.size(); ++i)
  {
    RCLCPP_INFO(rclcpp::get_logger("Ra6aSystem"),
      "J%ld | Pos: %.3f | Vel: %.3f",
      i,
      cmd_positions_[i],
      cmd_velocities_[i]);
  }

  if (hw_mode_ == "real")
  {
    // Send BOTH position and velocity to STM32 here
  }

  return hardware_interface::return_type::OK;
}

}  

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ra6a_hardware::Ra6aSystem,
  hardware_interface::SystemInterface)
