#ifndef JETANK_MOTOR_CONTROL__JETANK_SERIAL_HARDWARE_HPP_
#define JETANK_MOTOR_CONTROL__JETANK_SERIAL_HARDWARE_HPP_

#include <map>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "jetank_motor_control/feetech_bus.hpp"

namespace jetank_motor_control
{

/// ros2_control SystemInterface for the JeTank's Feetech serial bus servos.
///
/// The same `<ros2_control>` block is shared between sim and hardware (only the
/// `<plugin>` differs), so this interface must export interfaces for ALL joints
/// in the block — arm servos, the gripper, gripper_right (sim mimic), and the
/// four wheels. Joints carrying a `servo_id` param drive a real servo over the
/// Feetech bus; joints without one (gripper_right, wheels — the wheels are driven
/// on hardware by the separate robot_controller node) are "virtual": their state
/// simply mirrors the last command so claiming controllers stay satisfied.
class JetankSerialHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(JetankSerialHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

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
  /// A joint backed by a physical Feetech servo.
  struct ServoJoint
  {
    std::string name;
    uint8_t id = 0;
    double offset_ticks = FeetechBus::CENTER_TICKS;  // tick at 0 rad (SCS15: 512)
    double direction = 1.0;                           // +1 or -1
    // SCS15: 1024 counts over ~300 deg -> counts per radian.
    double ticks_per_rad =
      FeetechBus::POSITION_FULLSCALE /
      (FeetechBus::SPAN_DEGREES * 3.14159265358979323846 / 180.0);
    double prev_position = 0.0;                        // for finite-diff velocity
    bool prev_valid = false;
  };

  double rad_to_ticks(const ServoJoint & j, double rad) const;
  double ticks_to_rad(const ServoJoint & j, double ticks) const;

  FeetechBus bus_;
  std::string serial_port_ = "/dev/ttyTHS1";
  int baud_rate_ = 1000000;

  // Generic per-(joint,interface) storage. std::map gives stable element
  // addresses across inserts, which the exported Handles point into.
  std::map<std::string, std::map<std::string, double>> commands_;
  std::map<std::string, std::map<std::string, double>> states_;

  std::vector<ServoJoint> servo_joints_;  // subset of joints with a servo_id
};

}  // namespace jetank_motor_control

#endif  // JETANK_MOTOR_CONTROL__JETANK_SERIAL_HARDWARE_HPP_
