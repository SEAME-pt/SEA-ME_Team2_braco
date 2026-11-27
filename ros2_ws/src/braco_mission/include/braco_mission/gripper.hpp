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

// GripperBackend: the gripper calls of pick_and_place (issue #148), so the sequence is unit
// tested with a fake (no ROS), like ArmBackend for the arm. Two implementations, chosen by a
// launch argument:
//   Gazebo:   gripper_action_controller, control_msgs/action/ParallelGripperCommand
//   real arm: /arm/set_gripper (std_srvs/SetBool) on arm_driver; refused while a STOP is latched
//             or when the mission does not own the control (ADR-0004, ADR-0005)

#ifndef BRACO_MISSION__GRIPPER_HPP_
#define BRACO_MISSION__GRIPPER_HPP_

#include <string>

namespace braco_mission
{

class GripperBackend
{
public:
  virtual ~GripperBackend() = default;

  // Open the gripper and wait until it is open. False: `message` says why.
  virtual bool open(std::string & message) = 0;
  // Close the gripper and wait until it stops: on an object it stalls before closing fully,
  // which is a success. False: `message` says why.
  virtual bool close(std::string & message) = 0;
};

}  // namespace braco_mission

#endif  // BRACO_MISSION__GRIPPER_HPP_
