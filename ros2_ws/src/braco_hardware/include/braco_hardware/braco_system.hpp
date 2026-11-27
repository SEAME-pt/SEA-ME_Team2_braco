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

#include <array>
#include <atomic>
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

using JointValues = std::array<double, 6>;

// Servo speed (min_speed..100) for the fastest joint: speed 100 is the joint's max velocity
// (rad/s), with a margin for the arm's latency. A NaN velocity gives min_speed.
uint8_t speed_for_velocity(
  const JointValues & velocity, const JointValues & max_velocity, uint8_t min_speed);

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
  // on_configure(): /joint_states, /arm/control_owner and /arm/stopped from arm_driver
  void subscribe();
  void on_owner(const std_msgs::msg::String & msg);
  void on_stopped(const std_msgs::msg::Bool & msg);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  sensor_msgs::msg::JointState::ConstSharedPtr latest_joint_states_;
  std::mutex joint_states_mutex_;

  // write(): ALL_ANGLES to arm_driver's /arm/command as "mission" (ADR-0004)
  rclcpp::Publisher<braco_interfaces::msg::ArmCommand>::SharedPtr command_pub_;
  decltype(braco_interfaces::msg::ArmCommand::angles_deg) last_target_deg_{};
  int64_t target_changed_ns_{0};  // when last_target_deg_ last changed (0: none yet)
  uint8_t min_speed_{5};          // <param> min_speed of the URDF, 1..100
  JointValues max_velocity_{};    // rad/s at speed 100: the URDF's <limit velocity>, per joint
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
