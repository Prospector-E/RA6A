#ifndef RA6A_HARDWARE__RA6A_SYSTEM_HPP_
#define RA6A_HARDWARE__RA6A_SYSTEM_HPP_

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"
#include <libserial/SerialPort.h>
#include <thread>
#include <mutex>
#include <atomic>
#include <vector>
#include <memory>
#include <string>

using namespace LibSerial;

namespace ra6a_hardware
{

class Ra6aSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(Ra6aSystem)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface>  export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  std::string hw_mode_ = "fake";
  std::string serial_port_ = "/dev/ttyACM0";
  int baud_rate_ = 115200;

  // Joint state + commands
  std::vector<double> positions_;
  std::vector<double> velocities_;
  std::vector<double> commands_;
  std::vector<double> velocity_commands_;
  std::vector<double> smoothed_vel_;     // EMA filter for velocity smoothing
  std::vector<double> last_sent_commands_;

  // Gripper bridge: /gripper_angle topic → Topic 105
  std::atomic<int>  gripper_angle_{26};
  std::atomic<bool> gripper_changed_{false};
  int last_gripper_sent_ = 26;
  rclcpp::Node::SharedPtr gripper_node_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr gripper_sub_;
  rclcpp::executors::SingleThreadedExecutor::SharedPtr gripper_executor_;
  std::thread gripper_spin_thread_;

  // Serial
  std::unique_ptr<SerialPort> serial_;
  std::thread serial_thread_;
  std::mutex serial_mutex_;
  std::atomic<bool> serial_running_{false};

  void open_serial();
  void close_serial();
  void serial_read_loop();
  void process_rosserial_byte(uint8_t b);
  void process_rosserial_packet(uint16_t topic_id,
                                const std::vector<uint8_t> & payload);
  void send_rosserial_packet(uint16_t topic_id,
                             const std::vector<uint8_t> & payload);
  void send_joint_positions(const std::vector<double> & positions);
  void send_joint_position_velocity(const std::vector<double> & positions,
                                    const std::vector<double> & velocities);
  void send_gripper_angle(int angle);

  // Rosserial parser
  enum ParseState {
    WAIT_SYNC1, WAIT_SYNC2, LEN_LOW, LEN_HIGH, CHECKSUM1,
    TOPIC_LOW, TOPIC_HIGH, PAYLOAD, CHECKSUM2
  };
  ParseState rs_state_ = WAIT_SYNC1;
  uint16_t rs_length_ = 0;
  uint16_t rs_topic_ = 0;
  uint8_t  rs_chk1_ = 0;
  uint32_t rs_chk2_sum_ = 0;
  std::vector<uint8_t> rs_payload_;
};

}  // namespace ra6a_hardware

#endif  // RA6A_HARDWARE__RA6A_SYSTEM_HPP_
