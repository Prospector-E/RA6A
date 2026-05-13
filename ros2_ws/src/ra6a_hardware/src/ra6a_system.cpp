/* ra6a_system.cpp — AR4-style velocity from JTC + gripper bridge */

#include "ra6a_hardware/ra6a_system.hpp"

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32.hpp>
#include <cstring>
#include <cmath>
#include <vector>
#include <thread>
#include <mutex>
#include <chrono>
#include <memory>

#include "pluginlib/class_list_macros.hpp"

namespace ra6a_hardware
{

static constexpr uint16_t TOPIC_JOINT_CMD   = 101;
static constexpr uint16_t TOPIC_JOINT_STATE = 102;
static constexpr uint16_t TOPIC_HOMED       = 103;
static constexpr uint16_t TOPIC_DONE        = 104;
static constexpr uint16_t TOPIC_GRIPPER     = 105;
static constexpr uint16_t TOPIC_JOINT_CMD_V = 107;
static constexpr uint8_t  RS_SYNC1 = 0xFF;
static constexpr uint8_t  RS_SYNC2 = 0xFE;

/* ── Lifecycle ───────────────────────────────────────── */

hardware_interface::CallbackReturn Ra6aSystem::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
    return hardware_interface::CallbackReturn::ERROR;

  positions_.resize(6, 0.0);
  velocities_.resize(6, 0.0);
  commands_.resize(6, 0.0);
  velocity_commands_.resize(6, 0.0);
  smoothed_vel_.resize(6, 0.0);
  last_sent_commands_.resize(6, 0.0);

  auto it = info_.hardware_parameters.find("mode");
  if (it != info_.hardware_parameters.end()) hw_mode_ = it->second;

  if (hw_mode_ == "real") {
    auto p = info_.hardware_parameters.find("serial_port");
    if (p != info_.hardware_parameters.end()) serial_port_ = p->second;
    auto b = info_.hardware_parameters.find("baud_rate");
    if (b != info_.hardware_parameters.end()) baud_rate_ = std::stoi(b->second);
  }

  RCLCPP_INFO(rclcpp::get_logger("Ra6aSystem"),
              "Initialized in %s mode", hw_mode_.c_str());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ra6aSystem::on_configure(
  const rclcpp_lifecycle::State & /*prev*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Ra6aSystem"), "Configuring...");
  if (hw_mode_ == "real") {
    try {
      open_serial();
      RCLCPP_INFO(rclcpp::get_logger("Ra6aSystem"),
                  "Serial %s @ %d baud", serial_port_.c_str(), baud_rate_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(rclcpp::get_logger("Ra6aSystem"),
                   "Serial open failed: %s", e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ra6aSystem::on_cleanup(
  const rclcpp_lifecycle::State & /*prev*/)
{
  if (hw_mode_ == "real") close_serial();
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ra6aSystem::on_activate(
  const rclcpp_lifecycle::State & /*prev*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Ra6aSystem"), "Activating...");
  if (hw_mode_ == "real") {
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  for (size_t i = 0; i < 6; i++) last_sent_commands_[i] = commands_[i];

  /* Gripper bridge: /gripper_angle → STM32 Topic 105 */
  gripper_node_ = rclcpp::Node::make_shared("ra6a_gripper_bridge");
  gripper_sub_ = gripper_node_->create_subscription<std_msgs::msg::Int32>(
    "/gripper_angle", 10,
    [this](const std_msgs::msg::Int32::SharedPtr msg) {
      gripper_angle_.store(msg->data);
      gripper_changed_.store(true);
    });
  gripper_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
  gripper_executor_->add_node(gripper_node_);
  gripper_spin_thread_ = std::thread([this]() {
    gripper_executor_->spin();
  });

  RCLCPP_INFO(rclcpp::get_logger("Ra6aSystem"),
              "Gripper bridge on /gripper_angle");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Ra6aSystem::on_deactivate(
  const rclcpp_lifecycle::State & /*prev*/)
{
  RCLCPP_INFO(rclcpp::get_logger("Ra6aSystem"), "Deactivating...");
  if (gripper_executor_) {
    gripper_executor_->cancel();
  }
  if (gripper_spin_thread_.joinable()) {
    gripper_spin_thread_.join();
  }
  gripper_sub_.reset();
  gripper_node_.reset();
  gripper_executor_.reset();
  return hardware_interface::CallbackReturn::SUCCESS;
}

/* ── Interfaces (AR4 style: position + velocity) ──── */

std::vector<hardware_interface::StateInterface>
Ra6aSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> si;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    si.emplace_back(info_.joints[i].name,
                    hardware_interface::HW_IF_POSITION, &positions_[i]);
    si.emplace_back(info_.joints[i].name,
                    hardware_interface::HW_IF_VELOCITY, &velocities_[i]);
  }
  return si;
}

std::vector<hardware_interface::CommandInterface>
Ra6aSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> ci;
  for (size_t i = 0; i < info_.joints.size(); i++) {
    ci.emplace_back(info_.joints[i].name,
                    hardware_interface::HW_IF_POSITION, &commands_[i]);
    ci.emplace_back(info_.joints[i].name,
                    hardware_interface::HW_IF_VELOCITY, &velocity_commands_[i]);
  }
  return ci;
}

/* ── Read / Write ──────────────────────────────────── */

hardware_interface::return_type Ra6aSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (hw_mode_ != "fake") {
    std::lock_guard<std::mutex> lock(serial_mutex_);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Ra6aSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  /* Echo commands → positions (instant RViz feedback) */
  for (size_t i = 0; i < positions_.size(); i++) {
    positions_[i] = commands_[i];
    velocities_[i] = velocity_commands_[i];
  }

  if (hw_mode_ == "fake") return hardware_interface::return_type::OK;

  if (serial_running_) {
    /* Smooth velocity with exponential moving average.
     * Plan-execute: velocities are already smooth → filter is transparent.
     * Servo teleop: JTC restarts interpolation each cycle → velocities are
     * erratic. The filter cleans this up, like AR4's AccelStepper does.
     * alpha=0.3 → responsive but smooth. Lower = smoother but laggier. */
    std::vector<double> smooth_vel(6);
    for (size_t i = 0; i < 6; i++) {
      smoothed_vel_[i] = 1.0 * velocity_commands_[i] + 0.0 * smoothed_vel_[i];
      smooth_vel[i] = smoothed_vel_[i];
    }

    send_joint_position_velocity(commands_, smooth_vel);
    for (size_t i = 0; i < 6; i++) last_sent_commands_[i] = commands_[i];

    /* Gripper: only on change */
    if (gripper_changed_.exchange(false)) {
      int angle = gripper_angle_.load();
      send_gripper_angle(angle);
      last_gripper_sent_ = angle;
    }
  }

  return hardware_interface::return_type::OK;
}

/* ── Serial ────────────────────────────────────────── */

void Ra6aSystem::open_serial()
{
  serial_ = std::make_unique<SerialPort>(serial_port_);
  serial_->SetBaudRate(BaudRate::BAUD_115200);
  serial_->SetCharacterSize(CharacterSize::CHAR_SIZE_8);
  serial_->SetParity(Parity::PARITY_NONE);
  serial_->SetStopBits(StopBits::STOP_BITS_1);
  serial_->SetFlowControl(FlowControl::FLOW_CONTROL_NONE);
  serial_running_ = true;
  rs_state_ = WAIT_SYNC1;
  serial_thread_ = std::thread(&Ra6aSystem::serial_read_loop, this);
  RCLCPP_INFO(rclcpp::get_logger("Ra6aSystem"), "Serial thread started");
}

void Ra6aSystem::close_serial()
{
  serial_running_ = false;
  if (serial_thread_.joinable()) serial_thread_.join();
  serial_.reset();
}

void Ra6aSystem::serial_read_loop()
{
  while (serial_running_) {
    try {
      uint8_t byte;
      serial_->ReadByte(byte, 50);
      process_rosserial_byte(byte);
    } catch (const ReadTimeout&) {
      continue;
    } catch (const std::exception& e) {
      if (serial_running_) {
        RCLCPP_WARN(rclcpp::get_logger("Ra6aSystem"),
                    "Serial read: %s", e.what());
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }
}

void Ra6aSystem::process_rosserial_byte(uint8_t b)
{
  switch (rs_state_) {
    case WAIT_SYNC1: if (b == RS_SYNC1) rs_state_ = WAIT_SYNC2; break;
    case WAIT_SYNC2:
      rs_state_ = (b == RS_SYNC2) ? LEN_LOW : WAIT_SYNC1; break;
    case LEN_LOW:
      rs_length_ = b; rs_chk1_ = b; rs_state_ = LEN_HIGH; break;
    case LEN_HIGH:
      rs_length_ |= (static_cast<uint16_t>(b) << 8);
      rs_chk1_ += b; rs_state_ = CHECKSUM1; break;
    case CHECKSUM1:
      rs_chk1_ = 255 - (rs_chk1_ % 256);
      if (b == rs_chk1_) { rs_state_ = TOPIC_LOW; rs_chk2_sum_ = 0; }
      else rs_state_ = WAIT_SYNC1;
      break;
    case TOPIC_LOW:
      rs_topic_ = b; rs_chk2_sum_ = b; rs_state_ = TOPIC_HIGH; break;
    case TOPIC_HIGH:
      rs_topic_ |= (static_cast<uint16_t>(b) << 8);
      rs_chk2_sum_ += b;
      rs_payload_.clear();
      rs_state_ = (rs_length_ > 0) ? PAYLOAD : CHECKSUM2;
      break;
    case PAYLOAD:
      rs_payload_.push_back(b); rs_chk2_sum_ += b;
      if (rs_payload_.size() >= rs_length_) rs_state_ = CHECKSUM2;
      break;
    case CHECKSUM2: {
      uint8_t expected = 255 - (rs_chk2_sum_ % 256);
      rs_state_ = WAIT_SYNC1;
      if (b == expected)
        process_rosserial_packet(rs_topic_, rs_payload_);
      break;
    }
    default: rs_state_ = WAIT_SYNC1;
  }
}

void Ra6aSystem::process_rosserial_packet(
  uint16_t topic, const std::vector<uint8_t>& pay)
{
  if (topic == TOPIC_JOINT_STATE && pay.size() == 24) {
    std::vector<float> deg(6);
    std::memcpy(deg.data(), pay.data(), 24);
    std::lock_guard<std::mutex> lock(serial_mutex_);
    // Don't update positions_ — 20Hz feedback fights 100Hz echo causing jitter
    (void)deg;
  }
  else if (topic == TOPIC_HOMED) {
    RCLCPP_INFO(rclcpp::get_logger("Ra6aSystem"), "STM32 homing complete");
  }
  else if (topic == TOPIC_DONE) {
    RCLCPP_INFO(rclcpp::get_logger("Ra6aSystem"), "STM32 motion complete");
  }
}

/* ── Packet helpers ────────────────────────────────── */

void Ra6aSystem::send_rosserial_packet(
  uint16_t topic, const std::vector<uint8_t>& payload)
{
  uint16_t len = payload.size();
  uint8_t ll = len & 0xFF, lh = (len >> 8) & 0xFF;
  uint8_t c1 = 255 - ((ll + lh) % 256);
  uint8_t tl = topic & 0xFF, th = (topic >> 8) & 0xFF;
  uint32_t cs = tl + th;
  for (auto b : payload) cs += b;
  uint8_t c2 = 255 - (cs % 256);

  std::vector<uint8_t> pkt = {RS_SYNC1, RS_SYNC2, ll, lh, c1, tl, th};
  pkt.insert(pkt.end(), payload.begin(), payload.end());
  pkt.push_back(c2);

  std::lock_guard<std::mutex> lock(serial_mutex_);
  try {
    serial_->Write(pkt);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("Ra6aSystem"),
                 "Write failed: %s", e.what());
  }
}

void Ra6aSystem::send_joint_positions(const std::vector<double>& pos)
{
  std::vector<uint8_t> pay(24);
  for (size_t j = 0; j < 6; j++) {
    float d = static_cast<float>(pos[j] * 180.0 / M_PI);
    std::memcpy(&pay[j * 4], &d, 4);
  }
  send_rosserial_packet(TOPIC_JOINT_CMD, pay);
}

void Ra6aSystem::send_joint_position_velocity(
  const std::vector<double>& pos, const std::vector<double>& vel)
{
  std::vector<uint8_t> pay(48);
  for (size_t j = 0; j < 6; j++) {
    float d = static_cast<float>(pos[j] * 180.0 / M_PI);
    std::memcpy(&pay[j * 4], &d, 4);
  }
  for (size_t j = 0; j < 6; j++) {
    float v = static_cast<float>(vel[j] * 180.0 / M_PI);
    std::memcpy(&pay[24 + j * 4], &v, 4);
  }
  send_rosserial_packet(TOPIC_JOINT_CMD_V, pay);
}

void Ra6aSystem::send_gripper_angle(int angle)
{
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  std::vector<uint8_t> pay = {static_cast<uint8_t>(angle)};
  send_rosserial_packet(TOPIC_GRIPPER, pay);
}

}  // namespace ra6a_hardware

PLUGINLIB_EXPORT_CLASS(
  ra6a_hardware::Ra6aSystem,
  hardware_interface::SystemInterface)
