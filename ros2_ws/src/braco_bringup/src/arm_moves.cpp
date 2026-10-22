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

#include "braco_bringup/arm_moves.hpp"

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <thread>

#include "mycobot.hpp"

using namespace std::chrono_literals;
using braco_interfaces::msg::ArmCommand;
using braco_interfaces::srv::TakeControl;
using Clock = std::chrono::steady_clock;

namespace braco_bringup
{

namespace
{

constexpr double STEP_DEG = 15.0;      // max lead of a target over the real angles
constexpr auto RESEND = 100ms;         // well below the driver's 300 ms watchdog
constexpr auto STATE_TIMEOUT = 500ms;  // older /joint_states = no feedback
constexpr auto LOOP_PERIOD = 20ms;
constexpr auto CALL_TIMEOUT = 5s;
constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr const char * SOURCE = "script";   // ArmCommand.source and the control owner name

}  // namespace

ArmMoves::ArmMoves(const std::string & name)
: rclcpp::Node(name)
{
  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, [this](const sensor_msgs::msg::JointState & msg) {onJoints(msg);});
  command_pub_ = create_publisher<ArmCommand>("arm/command", 10);
  power_client_ = create_client<std_srvs::srv::Trigger>("arm/power_on");
  release_client_ = create_client<std_srvs::srv::Trigger>("arm/release_servos");
  rest_client_ = create_client<std_srvs::srv::SetBool>("arm/set_read_only");
  control_client_ = create_client<TakeControl>("arm/take_control");
  executor_.add_node(get_node_base_interface());
}

void ArmMoves::onJoints(const sensor_msgs::msg::JointState & msg)
{
  if (msg.position.size() < 6) {return;}
  Angles a{};
  for (size_t i = 0; i < a.size(); ++i) {
    a[i] = msg.position[i] * RAD_TO_DEG;
  }
  angles_ = a;
  angles_time_ = Clock::now();
}

std::optional<Angles> ArmMoves::angles() const
{
  if (!angles_ || Clock::now() - angles_time_ > STATE_TIMEOUT) {return std::nullopt;}
  return angles_;
}

bool ArmMoves::waitForDriver(std::chrono::duration<double> timeout)
{
  const auto end = Clock::now() + timeout;
  while (Clock::now() < end && rclcpp::ok()) {
    executor_.spin_some();
    if (angles() && power_client_->service_is_ready() && release_client_->service_is_ready() &&
      control_client_->service_is_ready())
    {
      return true;
    }
    std::this_thread::sleep_for(100ms);
  }
  RCLCPP_ERROR(get_logger(), "arm_driver not available after %.0f s", timeout.count());
  return false;
}

bool ArmMoves::callTrigger(
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client, const char * what)
{
  auto future = client->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
  if (executor_.spin_until_future_complete(future, CALL_TIMEOUT) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "%s failed: timeout", what);
    return false;
  }
  const auto result = future.get();
  if (!result->success) {
    RCLCPP_ERROR(get_logger(), "%s failed: %s", what, result->message.c_str());
    return false;
  }
  return true;
}

bool ArmMoves::takeControl()
{
  auto request = std::make_shared<TakeControl::Request>();
  request->source = SOURCE;
  request->force = true;
  auto future = control_client_->async_send_request(request);
  if (executor_.spin_until_future_complete(future, CALL_TIMEOUT) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_ERROR(get_logger(), "take control failed: timeout");
    return false;
  }
  const auto result = future.get();
  if (!result->success) {
    RCLCPP_ERROR(get_logger(), "take control failed: %s", result->message.c_str());
    return false;
  }
  RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
  has_control_ = true;
  return true;
}

void ArmMoves::releaseControl()
{
  if (!has_control_) {return;}
  has_control_ = false;
  auto request = std::make_shared<TakeControl::Request>();
  request->source = SOURCE;
  request->release = true;
  auto future = control_client_->async_send_request(request);
  if (executor_.spin_until_future_complete(future, CALL_TIMEOUT) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(get_logger(), "release control: no answer (arm_driver frees it when idle)");
  }
}

bool ArmMoves::powerOn()
{
  return callTrigger(power_client_, "power on");
}

bool ArmMoves::releaseServos()
{
  return callTrigger(release_client_, "release servos");
}

bool ArmMoves::setRest(bool on)
{
  if (!rest_client_->wait_for_service(2s)) {
    RCLCPP_WARN(get_logger(), "arm/set_read_only not available - rest mode not set");
    return false;
  }
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = on;
  auto future = rest_client_->async_send_request(request);
  if (executor_.spin_until_future_complete(future, CALL_TIMEOUT) !=
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_WARN(get_logger(), "rest mode not set: timeout");
    return false;
  }
  const auto result = future.get();
  if (!result->success) {
    RCLCPP_WARN(get_logger(), "rest mode not set: %s", result->message.c_str());
    return false;
  }
  return true;
}

bool ArmMoves::moveTo(
  const Angles & pose, int speed, double tolerance_deg, std::chrono::duration<double> timeout)
{
  const auto end = Clock::now() + timeout;
  auto next_send = Clock::now();
  while (Clock::now() < end && rclcpp::ok()) {
    executor_.spin_some();
    std::this_thread::sleep_for(LOOP_PERIOD);
    const auto current = angles();
    if (!current) {continue;}
    // Same fraction for every joint, so they all arrive together (nothing = arrived)
    const auto step = mycobot::stepToward(*current, pose, STEP_DEG, tolerance_deg);
    if (!step) {return true;}
    const auto now = Clock::now();
    if (now < next_send) {continue;}
    next_send = now + RESEND;
    ArmCommand command;
    command.type = ArmCommand::ALL_ANGLES;
    for (size_t i = 0; i < pose.size(); ++i) {
      command.angles_deg[i] = (*step)[i];
    }
    command.speed = static_cast<uint8_t>(speed);
    command.source = SOURCE;
    command_pub_->publish(command);
  }
  stop();
  return false;
}

void ArmMoves::stop()
{
  ArmCommand command;
  command.type = ArmCommand::STOP;
  command.source = SOURCE;
  command_pub_->publish(command);
}

void ArmMoves::wait(std::chrono::duration<double> duration)
{
  const auto end = Clock::now() + duration;
  while (Clock::now() < end && rclcpp::ok()) {
    executor_.spin_some();
    std::this_thread::sleep_for(50ms);
  }
}

}  // namespace braco_bringup
