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
#include <cmath>
#include <cstddef>
#include <iterator>
#include <stdexcept>
#include <string>
#include <thread>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

using braco_interfaces::msg::ArmCommand;

namespace braco_hardware
{

namespace
{

constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr const char * SOURCE = "mission";   // ArmCommand.source and the control owner name

}  // namespace

hardware_interface::CallbackReturn BracoSystem::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  // The parent reads the <ros2_control> block of the URDF and creates the state interfaces
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (!joints_are_valid() || !load_parameters()) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

bool BracoSystem::joints_are_valid()
{
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
      return false;
    }
  }

  // write() fills ArmCommand.angles_deg[i] from the i-th URDF joint (joint1..joint6, in order)
  if (get_hardware_info().joints.size() != ArmCommand().angles_deg.size()) {
    RCLCPP_ERROR(
      get_logger(), "expected %zu joints, the URDF has %zu", ArmCommand().angles_deg.size(),
      get_hardware_info().joints.size());
    return false;
  }
  return true;
}

bool BracoSystem::load_parameters()
{
  // Optional <param>s of the <hardware> block; without them, the defaults of the header
  const auto & params_in_urdf = get_hardware_info().hardware_parameters;
  double rate = publish_rate_hz_;
  int speed = speed_;
  try {
    if (params_in_urdf.count("publish_rate_hz")) {
      rate = std::stod(params_in_urdf.at("publish_rate_hz"));
    }
    if (params_in_urdf.count("speed")) {
      speed = std::stoi(params_in_urdf.at("speed"));
    }
  } catch (const std::exception &) {
    RCLCPP_ERROR(get_logger(), "publish_rate_hz and speed must be numbers");
    return false;
  }
  if (rate <= 0.0 || speed < 1 || speed > 100) {
    RCLCPP_ERROR(
      get_logger(), "publish_rate_hz must be > 0 and speed 1..100 (got %.1f, %d)", rate, speed);
    return false;
  }
  publish_rate_hz_ = rate;
  speed_ = static_cast<uint8_t>(speed);
  return true;
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
  // write() sends the controller's targets to arm_driver on its command topic (ADR-0004)
  command_pub_ = get_node()->create_publisher<ArmCommand>("/arm/command", 10);
  last_publish_ = rclcpp::Time(0, 0, RCL_ROS_TIME);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn BracoSystem::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  commanding_ = false;   // until arm_controller claims the position commands

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

hardware_interface::return_type BracoSystem::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  // Called when a controller claims or releases command interfaces. After arm_controller
  // releases ours they keep its last target, which write() must not keep sending.
  const auto has_ours = [this](const std::vector<std::string> & interfaces) {
      for (const auto & joint : get_hardware_info().joints) {
        const auto name = joint.name + "/" + hardware_interface::HW_IF_POSITION;
        if (std::find(interfaces.begin(), interfaces.end(), name) != interfaces.end()) {
          return true;
        }
      }
      return false;
    };
  if (has_ours(stop_interfaces)) {
    commanding_ = false;
  }
  if (has_ours(start_interfaces)) {
    commanding_ = true;
  }
  return hardware_interface::return_type::OK;
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
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (!commanding_) {
    return hardware_interface::return_type::OK;   // no controller: nothing to send
  }
  // arm_driver sends ~14 commands/s and keeps only the latest: no point publishing at the
  // loop's 100 Hz. Skip, never sleep: a sleep here would stall read() and the controllers too.
  if ((time - last_publish_).seconds() < 1.0 / publish_rate_hz_) {
    return hardware_interface::return_type::OK;
  }

  ArmCommand msg;
  if (!make_command(msg)) {
    return hardware_interface::return_type::OK;   // arm_controller has not written yet
  }
  // Also when the target has not changed: arm_driver stops the arm after command_timeout_ms
  // (300 ms) without a motion command. Not clamped here: arm_driver clamps to JOINT_LIMITS.
  command_pub_->publish(msg);
  last_publish_ = time;
  return hardware_interface::return_type::OK;
}

bool BracoSystem::make_command(ArmCommand & msg)
{
  msg.type = ArmCommand::ALL_ANGLES;
  msg.source = SOURCE;
  msg.speed = speed_;
  const auto & joints = get_hardware_info().joints;
  for (std::size_t i = 0; i < joints.size(); ++i) {
    const double position =
      get_command(joints[i].name + "/" + hardware_interface::HW_IF_POSITION);
    if (std::isnan(position)) {
      return false;   // NaN until the controller writes its first target
    }
    msg.angles_deg[i] = position * RAD_TO_DEG;
  }
  return true;
}

}  // namespace braco_hardware

PLUGINLIB_EXPORT_CLASS(braco_hardware::BracoSystem, hardware_interface::SystemInterface)
