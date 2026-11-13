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

#include "braco_mission/mission.hpp"

#include <cmath>
#include <cstdio>
#include <string>
#include <utility>

#include "braco_mission/control_guard.hpp"

namespace braco_mission
{

namespace
{

Outcome failed(std::string message, bool position_only = false)
{
  Outcome o;
  o.position_only = position_only;
  o.message = std::move(message);
  return o;
}

std::string describe(const PoseError & e)
{
  char text[96];
  if (std::isnan(e.orientation_rad)) {
    std::snprintf(text, sizeof(text), "position error %.1f mm (position only)",
      e.position_m * 1000.0);
  } else {
    std::snprintf(text, sizeof(text), "position error %.1f mm, orientation error %.2f deg",
      e.position_m * 1000.0, e.orientation_rad * 180.0 / M_PI);
  }
  return text;
}

}  // namespace

Outcome move_to_pose(
  ArmBackend & arm, const Pose & goal, bool position_only_fallback, const Settings & settings)
{
  const GoalKind kind = classify(goal);
  if (kind == GoalKind::Invalid) {
    return failed("goal refused: the pose is not finite, or its orientation is not a rotation");
  }
  Pose target = goal;
  bool position_only = kind == GoalKind::PositionOnly;
  if (!position_only) {target.orientation = normalized(goal.orientation);}

  ControlGuard guard(arm);
  if (settings.use_control) {
    arm.report("control");
    std::string why;
    if (!arm.take_control(why)) {return failed("control refused: " + why, position_only);}
    guard.hold();
  }

  arm.report("planning");
  std::string why;
  bool planned = arm.plan(target, position_only, why);
  if (!planned && !position_only && position_only_fallback) {
    arm.report("planning position only");
    position_only = true;
    planned = arm.plan(target, true, why);
  }
  if (!planned) {return failed("no plan: " + why, position_only);}

  arm.report("executing");
  if (!arm.execute(why)) {return failed("execution failed: " + why, position_only);}

  arm.report("measuring");
  const std::optional<Pose> measured = arm.measure();
  if (!measured) {return failed("moved, but the tcp pose is not available from TF", position_only);}

  Outcome o;
  o.position_only = position_only;
  o.error = pose_error(target, *measured, !position_only);
  o.success = within(o.error, settings.position_tolerance_m, settings.orientation_tolerance_rad);
  o.message = (o.success ? "reached: " : "outside the tolerance: ") + describe(o.error);
  return o;
}

}  // namespace braco_mission
