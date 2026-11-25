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

// pick_and_place: the stages of one goal (issue #148), built on move_to_pose and move_cartesian.
//
//   goal            a pick or place pose that is not a pose, or an approach height out of
//                   (0, MAX_APPROACH_HEIGHT_M], is refused before anything else
//   control         with `use_control` (the real arm): take the control as "mission" once and
//                   hold it for the whole sequence, arm_controller active (ADR-0004)
//   open            open the gripper
//   pre_grasp       move_to_pose `approach_height_m` above the pick pose (+z of the planning frame)
//   approach_pick   straight line down to the pick pose: its pose error is the pick error
//   close           close the gripper on the object
//   retreat_pick    straight line up, back above the pick pose
//   pre_place       move_to_pose above the place pose
//   approach_place  straight line down to the place pose: its pose error is the place error
//   release         open the gripper
//   retreat_place   straight line up, above the place pose
//
// The first stage that fails ends the goal: nothing moves after it, and it is `failed_stage`.
// A move outside the node's tolerances is a failure too. The control is released on every path.
// The steps inside a stage (planning, executing, measuring) are not reported: the feedback is
// the stage. A straight line keeps the tcp orientation it starts with, so a position-only pick
// or place goes down and up with the orientation the planner chose above it.
//
// No ROS: the arm is a CartesianBackend, the gripper a GripperBackend, both faked in
// test/test_pick_and_place.cpp; MoveToPoseNode implements them.

#ifndef BRACO_MISSION__PICK_AND_PLACE_HPP_
#define BRACO_MISSION__PICK_AND_PLACE_HPP_

#include <limits>
#include <string>

#include "braco_mission/cartesian.hpp"
#include "braco_mission/gripper.hpp"
#include "braco_mission/mission.hpp"
#include "braco_mission/pose_math.hpp"

namespace braco_mission
{

// Above this the pre-grasp leaves the myCobot 280's reach for most picks on the bench.
constexpr double MAX_APPROACH_HEIGHT_M = 0.15;

// The stages, as the STAGE_* constants of braco_interfaces/action/PickAndPlace.
namespace stage
{
constexpr const char * GOAL = "goal";
constexpr const char * CONTROL = "control";
constexpr const char * OPEN = "open";
constexpr const char * PRE_GRASP = "pre_grasp";
constexpr const char * APPROACH_PICK = "approach_pick";
constexpr const char * CLOSE = "close";
constexpr const char * RETREAT_PICK = "retreat_pick";
constexpr const char * PRE_PLACE = "pre_place";
constexpr const char * APPROACH_PLACE = "approach_place";
constexpr const char * RELEASE = "release";
constexpr const char * RETREAT_PLACE = "retreat_place";
}  // namespace stage

struct PickPlaceRequest
{
  Pose pick;                       // tcp at the grasp, planning frame; orientation all 0: position
  Pose place;                      // tcp where the object is let go
  double approach_height_m = 0.05;
  double eef_step_m = 0.005;       // the straight lines
};

struct PickPlaceOutcome
{
  bool success = false;
  std::string failed_stage;        // empty on success
  PoseError pick_error{std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  PoseError place_error{std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  std::string message;
};

// Why `request` cannot run; empty when it can.
std::string check_request(const PickPlaceRequest & request);

// `pose` moved `height` up along +z, its orientation kept.
Pose above(const Pose & pose, double height);

PickPlaceOutcome pick_and_place(
  CartesianBackend & arm, GripperBackend & gripper, const PickPlaceRequest & request,
  const Settings & settings);

}  // namespace braco_mission

#endif  // BRACO_MISSION__PICK_AND_PLACE_HPP_
