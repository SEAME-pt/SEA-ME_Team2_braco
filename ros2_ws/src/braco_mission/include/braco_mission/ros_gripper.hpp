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

// The two GripperBackends of the mission node (issue #148), chosen by its `gripper` parameter:
//
//   sim   ActionGripper: /gripper_action_controller/gripper_cmd
//         (control_msgs/action/ParallelGripperCommand), the Gazebo arm. A goal ends when the
//         master joint reaches the command, or stalls on an object (allow_stalling in
//         controllers.yaml): both are a success.
//   real  ServiceGripper: /arm/set_gripper (std_srvs/SetBool, true = close) on arm_driver. It
//         answers once the command is on the serial line, so the gripper still moves: wait
//         `settle_s` after it. Refused while a STOP is latched or in rest mode (ADR-0005).
//
// Called from a goal's thread while an executor spins the node (the futures are answered
// there), like the node's other service calls.

#ifndef BRACO_MISSION__ROS_GRIPPER_HPP_
#define BRACO_MISSION__ROS_GRIPPER_HPP_

#include <memory>
#include <string>

#include "braco_mission/gripper.hpp"
#include "control_msgs/action/parallel_gripper_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace braco_mission
{

struct GripperConfig
{
  std::string kind = "sim";                 // sim: gripper_action_controller; real: arm_driver
  std::string action = "/gripper_action_controller/gripper_cmd";   // sim
  std::string service = "/arm/set_gripper";                        // real
  std::string joint = "gripper_controller";   // sim: the master joint (controllers.yaml)
  double open_position = 0.15;              // sim: rad of the master joint, fully open
  double closed_position = -0.74;           // sim: fully closed; on an object it stalls before
  // Longest wait for an answer and for the motion. The Gazebo gripper moves at ~0.1 rad/s: a
  // close on an object takes ~4 s, an empty one (0.89 rad) ~9 s (#148)
  double timeout_s = 15.0;
  double settle_s = 1.5;                    // real: the gripper moves after set_gripper answers
};

class ActionGripper : public GripperBackend
{
public:
  using Command = control_msgs::action::ParallelGripperCommand;
  ActionGripper(rclcpp::Node & node, const GripperConfig & config);

  bool open(std::string & message) override;
  bool close(std::string & message) override;

private:
  bool command(double position, const std::string & what, std::string & message);

  GripperConfig config_;
  rclcpp_action::Client<Command>::SharedPtr client_;
};

class ServiceGripper : public GripperBackend
{
public:
  using SetBool = std_srvs::srv::SetBool;
  ServiceGripper(rclcpp::Node & node, const GripperConfig & config);

  bool open(std::string & message) override;
  bool close(std::string & message) override;

private:
  bool set(bool closed, const std::string & what, std::string & message);

  GripperConfig config_;
  rclcpp::Client<SetBool>::SharedPtr client_;
};

// The gripper `config.kind` names; throws std::invalid_argument for any other kind.
std::unique_ptr<GripperBackend> make_gripper(rclcpp::Node & node, const GripperConfig & config);

}  // namespace braco_mission

#endif  // BRACO_MISSION__ROS_GRIPPER_HPP_
