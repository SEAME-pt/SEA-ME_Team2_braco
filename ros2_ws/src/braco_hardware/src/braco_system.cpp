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

#include "braco_hardware/braco_system.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <iterator>
#include <string>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace braco_hardware
{

hardware_interface::CallbackReturn BracoSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  // The parent reads the <ros2_control> block of the URDF and creates the state interfaces
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // read() fills position and velocity of every joint: refuse a URDF without them
  for (const auto & joint : get_hardware_info().joints) {
    bool has_position = false;
    bool has_velocity = false;
    for (const auto & state : joint.state_interfaces) {
      has_position = has_position || state.name == hardware_interface::HW_IF_POSITION;
      has_velocity = has_velocity || state.name == hardware_interface::HW_IF_VELOCITY;
    }
    if (!has_position || !has_velocity) {
      RCLCPP_ERROR(
        get_logger(), "joint %s needs position and velocity state interfaces",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BracoSystem::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!get_node()) {
    RCLCPP_ERROR(get_logger(), "no node from the controller manager");
    return hardware_interface::CallbackReturn::ERROR;
  }
  {
    std::lock_guard<std::mutex> lock(joint_states_mutex_);
    latest_joint_states_.reset();
  }
  // arm_driver publishes /joint_states (ADR-0004): keep only the latest message for read()
  joint_states_sub_ = get_node()->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      std::lock_guard<std::mutex> lock(joint_states_mutex_);
      latest_joint_states_ = msg;
    });
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BracoSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // read() runs right after this, too soon for the first /joint_states to have arrived, and a
  // read() without one stops the hardware. So wait here (up to 2 s) until it arrives: the
  // controller manager delivers it to our subscription on another thread while we wait.
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(2);
  while (std::chrono::steady_clock::now() < deadline) {
    {
      std::lock_guard<std::mutex> lock(joint_states_mutex_);
      if (latest_joint_states_) {
        return hardware_interface::CallbackReturn::SUCCESS;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  RCLCPP_ERROR(get_logger(), "no /joint_states in 2 s: is arm_driver running?");
  return hardware_interface::CallbackReturn::ERROR;
}

hardware_interface::return_type BracoSystem::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  sensor_msgs::msg::JointState::ConstSharedPtr msg;
  {
    std::lock_guard<std::mutex> lock(joint_states_mutex_);
    msg = latest_joint_states_;
  }
  if (!msg) {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 1000, "no /joint_states from arm_driver yet");
    return hardware_interface::return_type::ERROR;
  }

  // The message lists the joints by name, in any order: find each URDF joint in it
  for (const auto & joint : get_hardware_info().joints) {
    const auto it = std::find(msg->name.begin(), msg->name.end(), joint.name);
    const auto i = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
    if (it == msg->name.end() || i >= msg->position.size()) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "/joint_states has no position for %s",
        joint.name.c_str());
      return hardware_interface::return_type::ERROR;
    }
    const double velocity = i < msg->velocity.size() ? msg->velocity[i] : 0.0;
    set_state(joint.name + "/" + hardware_interface::HW_IF_POSITION, msg->position[i]);
    set_state(joint.name + "/" + hardware_interface::HW_IF_VELOCITY, velocity);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type BracoSystem::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read-only for now: sending commands to arm_driver is #101
  return hardware_interface::return_type::OK;
}

}  // namespace braco_hardware

PLUGINLIB_EXPORT_CLASS(braco_hardware::BracoSystem, hardware_interface::SystemInterface)
