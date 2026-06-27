#include "jetank_motor_control/jetank_serial_hardware.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr const char * kLogger = "JetankSerialHardware";
}

namespace jetank_motor_control
{

hardware_interface::CallbackReturn JetankSerialHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Connection parameters (declared in the <hardware> block of the xacro).
  auto it = info_.hardware_parameters.find("serial_port");
  if (it != info_.hardware_parameters.end()) {
    serial_port_ = it->second;
  }
  it = info_.hardware_parameters.find("baud_rate");
  if (it != info_.hardware_parameters.end()) {
    try {
      baud_rate_ = std::stoi(it->second);
    } catch (const std::exception &) {
      RCLCPP_WARN(
        rclcpp::get_logger(kLogger), "Invalid baud_rate '%s'; using %d",
        it->second.c_str(), baud_rate_);
    }
  }

  // Allocate generic storage for every declared interface and identify the
  // joints backed by a real servo (those carrying a `servo_id` param).
  for (const auto & joint : info_.joints) {
    for (const auto & ci : joint.command_interfaces) {
      commands_[joint.name][ci.name] = std::numeric_limits<double>::quiet_NaN();
    }
    for (const auto & si : joint.state_interfaces) {
      states_[joint.name][si.name] = 0.0;
    }

    const auto sid = joint.parameters.find("servo_id");
    if (sid == joint.parameters.end()) {
      continue;  // virtual joint (gripper_right / wheels) — mirror in read()
    }

    ServoJoint sj;
    sj.name = joint.name;
    try {
      sj.id = static_cast<uint8_t>(std::stoi(sid->second));
    } catch (const std::exception &) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLogger), "Joint '%s' has invalid servo_id '%s'",
        joint.name.c_str(), sid->second.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Optional per-joint calibration params (defaults are sensible for an
    // uncalibrated SMS/STS servo; real values come from the bench calibration).
    auto get_param = [&joint](const std::string & key, double fallback) {
      const auto p = joint.parameters.find(key);
      if (p == joint.parameters.end()) {
        return fallback;
      }
      try {
        return std::stod(p->second);
      } catch (const std::exception &) {
        return fallback;
      }
    };
    sj.offset_ticks = get_param("offset", sj.offset_ticks);
    sj.direction = get_param("direction", sj.direction);
    sj.ticks_per_rad = get_param("ticks_per_rad", sj.ticks_per_rad);

    servo_joints_.push_back(sj);
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLogger),
    "Initialized: %zu joints (%zu real servos) on %s @ %d baud",
    info_.joints.size(), servo_joints_.size(), serial_port_.c_str(), baud_rate_);

  return hardware_interface::CallbackReturn::SUCCESS;
}

double JetankSerialHardware::rad_to_ticks(const ServoJoint & j, double rad) const
{
  return j.offset_ticks + j.direction * rad * j.ticks_per_rad;
}

double JetankSerialHardware::ticks_to_rad(const ServoJoint & j, double ticks) const
{
  return (ticks - j.offset_ticks) / (j.direction * j.ticks_per_rad);
}

hardware_interface::CallbackReturn JetankSerialHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!bus_.open(serial_port_, baud_rate_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogger), "Failed to open serial port %s",
      serial_port_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Ping each servo, enable torque, and seed the command with the current
  // position so the controller does not jerk the arm on the first write.
  for (auto & sj : servo_joints_) {
    if (!bus_.ping(sj.id)) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLogger), "Servo id %u (%s) did not respond to ping",
        sj.id, sj.name.c_str());
      bus_.close();
      return hardware_interface::CallbackReturn::ERROR;
    }
    uint16_t ticks = 0;
    if (bus_.read_present_position(sj.id, ticks)) {
      const double rad = ticks_to_rad(sj, static_cast<double>(ticks));
      states_[sj.name]["position"] = rad;
      commands_[sj.name]["position"] = rad;
      sj.prev_position = rad;
      sj.prev_valid = true;
    }
    bus_.set_torque(sj.id, true);
  }

  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Activated — torque enabled on all servos");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JetankSerialHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (const auto & sj : servo_joints_) {
    bus_.set_torque(sj.id, false);
  }
  bus_.close();
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "Deactivated — torque disabled, port closed");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
JetankSerialHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (auto & joint : states_) {
    for (auto & iface : joint.second) {
      interfaces.emplace_back(joint.first, iface.first, &iface.second);
    }
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
JetankSerialHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (auto & joint : commands_) {
    for (auto & iface : joint.second) {
      interfaces.emplace_back(joint.first, iface.first, &iface.second);
    }
  }
  return interfaces;
}

hardware_interface::return_type JetankSerialHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  const double dt = period.seconds();

  // Real servos: read present position, derive velocity by finite difference.
  for (auto & sj : servo_joints_) {
    uint16_t ticks = 0;
    if (!bus_.read_present_position(sj.id, ticks)) {
      // Keep the last good state on a transient read miss rather than fault.
      continue;
    }
    const double rad = ticks_to_rad(sj, static_cast<double>(ticks));
    states_[sj.name]["position"] = rad;
    if (states_[sj.name].count("velocity")) {
      states_[sj.name]["velocity"] =
        (sj.prev_valid && dt > 1e-6) ? (rad - sj.prev_position) / dt : 0.0;
    }
    sj.prev_position = rad;
    sj.prev_valid = true;
  }

  // Virtual joints (gripper_right, wheels): mirror command -> state so any
  // claiming controller sees the joint "following" perfectly.
  for (auto & joint : states_) {
    const bool is_servo = std::any_of(
      servo_joints_.begin(), servo_joints_.end(),
      [&joint](const ServoJoint & s) {return s.name == joint.first;});
    if (is_servo) {
      continue;
    }
    const auto cmd_it = commands_.find(joint.first);
    if (cmd_it == commands_.end()) {
      continue;
    }
    for (auto & iface : joint.second) {
      const auto c = cmd_it->second.find(iface.first);
      if (c != cmd_it->second.end() && std::isfinite(c->second)) {
        iface.second = c->second;
      }
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JetankSerialHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (const auto & sj : servo_joints_) {
    const auto j = commands_.find(sj.name);
    if (j == commands_.end()) {
      continue;
    }
    const auto c = j->second.find("position");
    if (c == j->second.end() || !std::isfinite(c->second)) {
      continue;  // no valid position command yet
    }
    double ticks = rad_to_ticks(sj, c->second);
    ticks = std::clamp(ticks, 0.0, static_cast<double>(FeetechBus::TICKS_PER_REV - 1));
    bus_.write_goal_position(sj.id, static_cast<uint16_t>(std::lround(ticks)));
  }
  return hardware_interface::return_type::OK;
}

}  // namespace jetank_motor_control

PLUGINLIB_EXPORT_CLASS(
  jetank_motor_control::JetankSerialHardware, hardware_interface::SystemInterface)
