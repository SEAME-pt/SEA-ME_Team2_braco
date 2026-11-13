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

// move_cartesian: the tcp moves in straight lines through waypoints (issue #85), the steps
// of one goal in the order of ADR-0004.
//
//   1. a request that cannot be planned (no waypoints, a waypoint that is not a pose, two
//      waypoints at the same place, eef_step or min_fraction out of range) is refused first
//   2. with `use_control` (the real arm): take the control as "mission", without force; then
//      braco_hardware and arm_controller active again, if a STOP or a lost control turned them off
//   3. a waypoint with its orientation all 0 keeps the tcp orientation measured at the start
//   4. plan the straight-line path (computeCartesianPath): an error (fraction -1) or less
//      than `min_fraction` of the path planned -> nothing is executed
//   5. execute, then measure the tcp and compare it with the last waypoint
//   6. the control taken in 2 is released on every path, also on failure
//
// The planned tcp path is compared with the ideal polyline (start -> waypoints): its largest
// distance from the lines is reported as `max_deviation_m`.
//
// No ROS: every ROS call is behind CartesianBackend, unit tested with a fake
// (test/test_cartesian.cpp); move_to_pose_node.cpp implements it with MoveIt 2 and TF.

#ifndef BRACO_MISSION__CARTESIAN_HPP_
#define BRACO_MISSION__CARTESIAN_HPP_

#include <limits>
#include <string>
#include <vector>

#include "braco_mission/mission.hpp"
#include "braco_mission/pose_math.hpp"

namespace braco_mission
{

// eef_step (metres between two tcp points of the path) above this would cut corners.
constexpr double MAX_EEF_STEP_M = 0.05;
// Two consecutive waypoints closer than this are the same point: a typo, not a line.
constexpr double MIN_WAYPOINT_SPACING_M = 1e-4;

struct CartesianPlan
{
  double fraction = -1.0;       // share of the path planned, 0..1; -1: planning error
  std::vector<Vec3> tcp_path;   // the tcp at each point of the planned trajectory
};

class CartesianBackend : public ArmBackend
{
public:
  // Plan straight lines from the current state through `waypoints` (planning frame), a tcp
  // point every `eef_step` metres, time-parameterised with the node's velocity scaling. The
  // plan is kept for execute(). On an error `message` says why.
  virtual CartesianPlan plan_cartesian(
    const std::vector<Pose> & waypoints, double eef_step, std::string & message) = 0;
};

struct CartesianRequest
{
  std::vector<Pose> waypoints;   // orientation all 0: keep the tcp orientation at the start
  double eef_step_m = 0.005;
  double min_fraction = 1.0;     // below it the path is not executed
};

struct CartesianOutcome
{
  bool success = false;
  double fraction = std::numeric_limits<double>::quiet_NaN();
  double max_deviation_m = std::numeric_limits<double>::quiet_NaN();
  PoseError error{std::numeric_limits<double>::quiet_NaN(),
    std::numeric_limits<double>::quiet_NaN()};
  std::string message;
};

// Why `request` cannot be planned; empty when it can.
std::string check_request(const CartesianRequest & request);

// The waypoints with every orientation a unit quaternion: all 0 takes `start_orientation`.
std::vector<Pose> fill_orientation(const std::vector<Pose> & waypoints, const Quat & start);

// Distance from `p` to the segment a-b (to a when a and b are the same point).
double distance_to_segment(const Vec3 & p, const Vec3 & a, const Vec3 & b);

// The largest distance from a point of `points` to the polyline; NaN when either is empty.
double max_deviation(const std::vector<Vec3> & polyline, const std::vector<Vec3> & points);

CartesianOutcome move_cartesian(
  CartesianBackend & arm, const CartesianRequest & request, const Settings & settings);

}  // namespace braco_mission

#endif  // BRACO_MISSION__CARTESIAN_HPP_
