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

// MotionGroup: the MoveIt 2 calls of MoveToPoseNode, so tests can fake them (no ROS, no MoveIt).
// MoveItMotionGroup (moveit_motion_group.hpp) implements it with MoveGroupInterface.

#ifndef BRACO_MISSION__MOTION_GROUP_HPP_
#define BRACO_MISSION__MOTION_GROUP_HPP_

#include <string>
#include <vector>

#include "braco_mission/cartesian.hpp"
#include "braco_mission/pose_math.hpp"

namespace braco_mission
{

struct MotionConfig
{
  std::string group = "arm";
  std::string end_effector = "tcp";
  double velocity_scaling = 0.1;       // ADR-0004
  double acceleration_scaling = 0.1;   // ADR-0004
  double planning_time_s = 5.0;
  double state_timeout_s = 1.0;        // wait for the current robot state
};

class MotionGroup
{
public:
  virtual ~MotionGroup() = default;

  virtual std::string planning_frame() const = 0;
  // Plan from the current state to `goal`, its position only when `position_only`.
  virtual bool plan(const Pose & goal, bool position_only) = 0;
  // Straight lines through `waypoints`, timed with the scaling; fraction < 0: `message` says why.
  virtual CartesianPlan plan_cartesian(
    const std::vector<Pose> & waypoints, double eef_step, std::string & message) = 0;
  // Execute the last plan of either kind.
  virtual bool execute() = 0;
  virtual void stop() = 0;
};

}  // namespace braco_mission

#endif  // BRACO_MISSION__MOTION_GROUP_HPP_
