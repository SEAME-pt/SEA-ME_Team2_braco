// Copyright 2026 SEA:ME Team 2
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef BRACO_HARDWARE__BRACO_SYSTEM_HPP_
#define BRACO_HARDWARE__BRACO_SYSTEM_HPP_

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#include "braco_interfaces/msg/arm_command.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

namespace braco_hardware
{

// Firmware speed setting (1..100) for a joint velocity: measured linear, ~0.0265 rad/s per unit
// on every joint (docs/hardware/urdf-sim-to-real.md, "Measured joint speed per firmware speed
// setting").
constexpr double RAD_S_PER_SPEED_UNIT = 0.0265;
// Above the target's own velocity, so a servo that fell behind (start-up, dead zone) catches up
constexpr double SPEED_MARGIN = 1.5;
// Never slower than this (~15 deg/s): the arm still closes a few degrees of lag after the last
// point
constexpr int MIN_SPEED = 10;

// The speed setting that moves a joint at `rad_per_s` (times SPEED_MARGIN), between MIN_SPEED and
// `max_speed` (the URDF's <param> speed). A fixed high speed makes the servo rush to each ~70 ms
// setpoint and stop until the next one: the arm moves in jerks (stop-and-go).
inline uint8_t speed_for_velocity(double rad_per_s, uint8_t max_speed)
{
  const double wanted = std::ceil(std::abs(rad_per_s) * SPEED_MARGIN / RAD_S_PER_SPEED_UNIT);
  const int low = std::min<int>(MIN_SPEED, max_speed);
  return static_cast<uint8_t>(
    std::clamp(static_cast<int>(std::min(wanted, 100.0)), low, static_cast<int>(max_speed)));
}

// The real arm for ros2_control (ADR-0004): reads the joints from arm_driver's /joint_states and
// sends arm_controller's targets on /arm/command. Never opens the serial port: arm_driver owns it.
class BracoSystem : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type perform_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // on_init() steps; each logs why it fails
  bool joints_are_valid();
  bool load_parameters();
  // write(): ALL_ANGLES from the position commands; false while any is still NaN
  bool make_command(braco_interfaces::msg::ArmCommand & msg);
  // write(): true once the target has not changed for HOLD_S (the arm is holding it)
  bool target_is_idle(const braco_interfaces::msg::ArmCommand & msg, int64_t now_ns);
  // write(): the speed for this target, from how fast it moved since the last one sent
  uint8_t command_speed(const braco_interfaces::msg::ArmCommand & msg, int64_t now_ns);
  // on_configure(): /joint_states, /arm/control_owner and /arm/stopped from arm_driver
  void subscribe();
  void on_owner(const std_msgs::msg::String & msg);
  void on_stopped(const std_msgs::msg::Bool & msg);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  sensor_msgs::msg::JointState::ConstSharedPtr latest_joint_states_;
  std::mutex joint_states_mutex_;

  // write(): ALL_ANGLES to arm_driver's /arm/command as "mission" (ADR-0004)
  rclcpp::Publisher<braco_interfaces::msg::ArmCommand>::SharedPtr command_pub_;
  int64_t last_publish_ns_{0};    // last publish, on the controller manager's clock (ns)
  decltype(braco_interfaces::msg::ArmCommand::angles_deg) last_target_deg_{};
  int64_t target_changed_ns_{0};  // when last_target_deg_ last changed (0: none yet)
  // 50 Hz: arm_driver takes the latest every ~70 ms, so it gets a target at most 20 ms old
  // (at 20 Hz, 0 or 50 ms: the step between two sent targets jumped between 1x and 2x)
  double publish_rate_hz_{50.0};  // <param> publish_rate_hz of the URDF
  uint8_t speed_{50};             // <param> speed of the URDF, 1..100: the highest speed sent
  // command_speed(): the last target sent, when, and its speed (has_sent_ false: none yet)
  decltype(braco_interfaces::msg::ArmCommand::angles_deg) sent_deg_{};
  int64_t sent_ns_{0};
  uint8_t sent_speed_{MIN_SPEED};
  bool has_sent_{false};
  std::atomic<bool> commanding_{false};  // a controller holds our position commands

  // The control owner (ADR-0002): write() sends only while it is "mission", and aborts the goal
  // on a STOP (ADR-0005 latch) or when another client takes the control from mission
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr owner_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr stopped_sub_;
  std::atomic<bool> owner_is_mission_{false};
  std::atomic<bool> stop_seen_{false};
  std::atomic<bool> lost_control_{false};
};

}  // namespace braco_hardware

#endif  // BRACO_HARDWARE__BRACO_SYSTEM_HPP_
