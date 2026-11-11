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

namespace braco_hardware
{

// The real arm for ros2_control (ADR-0004): reads the joints from arm_driver's /joint_states.
// Never opens the serial port: arm_driver is its only owner.
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

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
  sensor_msgs::msg::JointState::ConstSharedPtr latest_joint_states_;
  std::mutex joint_states_mutex_;

  // write(): ALL_ANGLES to arm_driver's /arm/command as "mission" (ADR-0004)
  rclcpp::Publisher<braco_interfaces::msg::ArmCommand>::SharedPtr command_pub_;
  rclcpp::Time last_publish_{0, 0, RCL_ROS_TIME};
  double publish_rate_hz_{20.0};  // <param> publish_rate_hz of the URDF
  uint8_t speed_{50};             // <param> speed of the URDF, 1..100
  std::atomic<bool> commanding_{false};  // a controller holds our position commands
};

}  // namespace braco_hardware

#endif  // BRACO_HARDWARE__BRACO_SYSTEM_HPP_
