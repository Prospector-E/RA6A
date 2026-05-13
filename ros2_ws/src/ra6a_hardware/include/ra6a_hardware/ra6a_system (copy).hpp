#ifndef RA6A_SYSTEM_HPP
#define RA6A_SYSTEM_HPP

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include <vector>
#include <string>

namespace ra6a_hardware
{

class Ra6aSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Ra6aSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  std::string hw_mode_;

  // STATES
  std::vector<double> positions_;
  std::vector<double> velocities_;
 

  // COMMANDS
  std::vector<double> cmd_positions_;
  std::vector<double> cmd_velocities_;
 
};

}

#endif
