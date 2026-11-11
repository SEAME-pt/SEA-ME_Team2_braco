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

// move_to_pose: the steps of one pose goal, in the order of ADR-0004.
//
//   1. a goal that is not a pose (not finite, not a rotation) is refused before anything else
//   2. with `use_control` (the real arm): take the control as "mission", without force;
//      refused -> the goal is not sent, nothing is queued
//   3. plan the full pose; no plan and `position_only_fallback` -> plan the position only
//   4. execute (arm_controller), then measure the tcp from TF and compare it with the goal
//   5. the control taken in 2 is released on every path, also on failure
//
// Every ROS call is behind ArmBackend, so the steps are unit tested with a fake
// (test/test_mission.cpp); move_to_pose_node.cpp implements it with MoveIt 2 and TF.

#ifndef BRACO_MISSION__MISSION_HPP_
#define BRACO_MISSION__MISSION_HPP_

#include <optional>   // cpplint takes it for a C header: it must come first

#include <limits>
#include <string>

#include "braco_mission/pose_math.hpp"

namespace braco_mission
{

class ArmBackend
{
public:
  virtual ~ArmBackend() = default;

  // /arm/take_control as "mission", no force (ADR-0004). False: `message` says why.
  virtual bool take_control(std::string & message) = 0;
  virtual void release_control() = 0;
  // Plan to `goal` (planning frame), its position only when `position_only`.
  virtual bool plan(const Pose & goal, bool position_only, std::string & message) = 0;
  // Execute the last plan.
  virtual bool execute(std::string & message) = 0;
  // The tcp in the planning frame, from TF; nothing if it is not available.
  virtual std::optional<Pose> measure() = 0;
  // Feedback: the step the goal is in.
  virtual void report(const std::string & stage) = 0;
};

struct Settings
{
  bool use_control = false;               // the real arm: take the control (ADR-0004)
  double position_tolerance_m = 0.005;
  double orientation_tolerance_rad = 0.035;   // 2 degrees
};

struct Outcome
{
  bool success = false;
  PoseError error{std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  bool position_only = false;
  std::string message;
};

Outcome move_to_pose(
  ArmBackend & arm, const Pose & goal, bool position_only_fallback, const Settings & settings);

}  // namespace braco_mission

#endif  // BRACO_MISSION__MISSION_HPP_
