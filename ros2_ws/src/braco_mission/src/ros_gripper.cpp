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

// ActionGripper and ServiceGripper (include/braco_mission/ros_gripper.hpp).

#include "braco_mission/ros_gripper.hpp"

#include <chrono>
#include <future>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

using namespace std::chrono_literals;

namespace braco_mission
{

namespace
{

std::chrono::duration<double> seconds(double s)
{
  return std::chrono::duration<double>(s);
}

const char * code_name(rclcpp_action::ResultCode code)
{
  switch (code) {
    case rclcpp_action::ResultCode::ABORTED: return "aborted";
    case rclcpp_action::ResultCode::CANCELED: return "canceled";
    default: return "no result";
  }
}

}  // namespace

// ---- ActionGripper ---------------------------------------------------------------------

ActionGripper::ActionGripper(rclcpp::Node & node, const GripperConfig & config)
: config_(config), client_(rclcpp_action::create_client<Command>(&node, ACTION))
{
}

bool ActionGripper::open(std::string & message)
{
  return command(config_.open_position, "open gripper", message);
}

bool ActionGripper::close(std::string & message)
{
  return command(config_.closed_position, "close gripper", message);
}

bool ActionGripper::command(double position, const std::string & what, std::string & message)
{
  if (!client_->wait_for_action_server(2s)) {
    message = what + ": " + ACTION + " is not available";
    return false;
  }
  Command::Goal goal;
  goal.command.name = {config_.joint};
  goal.command.position = {position};
  auto sent = client_->async_send_goal(goal);
  if (sent.wait_for(seconds(config_.timeout_s)) != std::future_status::ready) {
    message = what + ": gripper_action_controller did not answer";
    return false;
  }
  const auto handle = sent.get();
  if (!handle) {
    message = what + ": refused by gripper_action_controller";
    return false;
  }
  auto done = client_->async_get_result(handle);
  if (done.wait_for(seconds(config_.timeout_s)) != std::future_status::ready) {
    client_->async_cancel_goal(handle);
    message = what + ": not done in " + std::to_string(config_.timeout_s) + " s";
    return false;
  }
  const auto result = done.get();
  if (result.code != rclcpp_action::ResultCode::SUCCEEDED) {
    message = what + ": " + code_name(result.code) + " by gripper_action_controller";
    return false;
  }
  return true;
}

// ---- ServiceGripper --------------------------------------------------------------------

ServiceGripper::ServiceGripper(rclcpp::Node & node, const GripperConfig & config)
: config_(config), client_(node.create_client<SetBool>(SERVICE))
{
}

bool ServiceGripper::open(std::string & message)
{
  return set(false, "open gripper", message);
}

bool ServiceGripper::close(std::string & message)
{
  return set(true, "close gripper", message);
}

bool ServiceGripper::set(bool closed, const std::string & what, std::string & message)
{
  if (!client_->wait_for_service(2s)) {
    message = what + ": " + SERVICE + " is not available";
    return false;
  }
  auto request = std::make_shared<SetBool::Request>();
  request->data = closed;
  auto future = client_->async_send_request(request);
  if (future.wait_for(seconds(config_.timeout_s)) != std::future_status::ready) {
    message = what + ": arm_driver did not answer";
    return false;
  }
  const auto response = future.get();
  if (!response->success) {
    message = response->message;   // arm_driver says why: STOP latched, rest mode, serial
    return false;
  }
  std::this_thread::sleep_for(seconds(config_.settle_s));   // the gripper is still moving
  return true;
}

// ---- factory ---------------------------------------------------------------------------

std::unique_ptr<GripperBackend> make_gripper(rclcpp::Node & node, const GripperConfig & config)
{
  if (config.kind == "sim") {return std::make_unique<ActionGripper>(node, config);}
  if (config.kind == "real") {return std::make_unique<ServiceGripper>(node, config);}
  throw std::invalid_argument("gripper must be sim or real, not '" + config.kind + "'");
}

}  // namespace braco_mission
