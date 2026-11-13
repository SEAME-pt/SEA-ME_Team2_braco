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

#include "braco_mission/cartesian.hpp"

#include <optional>   // cpplint takes it for a C header: it must come first

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <limits>
#include <string>
#include <utility>
#include <vector>

#include "braco_mission/control_guard.hpp"

namespace braco_mission
{

namespace
{

CartesianOutcome failed(std::string message)
{
  CartesianOutcome o;
  o.message = std::move(message);
  return o;
}

double dot(const Vec3 & a, const Vec3 & b)
{
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 minus(const Vec3 & a, const Vec3 & b)
{
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

std::string describe(const CartesianOutcome & o)
{
  char text[128];
  std::snprintf(text, sizeof(text),
    "position error %.1f mm, orientation error %.2f deg, line deviation %.1f mm",
    o.error.position_m * 1000.0, o.error.orientation_rad * 180.0 / M_PI,
    o.max_deviation_m * 1000.0);
  return text;
}

}  // namespace

std::string check_request(const CartesianRequest & request)
{
  if (request.waypoints.empty()) {return "no waypoints";}
  if (!(request.eef_step_m > 0.0 && request.eef_step_m <= MAX_EEF_STEP_M)) {
    return "eef_step must be in (0, " + std::to_string(MAX_EEF_STEP_M) + "] m";
  }
  if (!(request.min_fraction > 0.0 && request.min_fraction <= 1.0)) {
    return "min_fraction must be in (0, 1]";
  }
  for (std::size_t i = 0; i < request.waypoints.size(); ++i) {
    const Pose & w = request.waypoints[i];
    if (classify(w) == GoalKind::Invalid) {
      return "waypoint " + std::to_string(i) +
             " is not finite, or its orientation is not a rotation";
    }
    if (i > 0 &&
      position_error(w.position, request.waypoints[i - 1].position) < MIN_WAYPOINT_SPACING_M)
    {
      return "waypoint " + std::to_string(i) + " is at the same place as waypoint " +
             std::to_string(i - 1);
    }
  }
  return "";
}

std::vector<Pose> fill_orientation(const std::vector<Pose> & waypoints, const Quat & start)
{
  std::vector<Pose> out = waypoints;
  for (Pose & w : out) {
    w.orientation =
      classify(w) == GoalKind::PositionOnly ? normalized(start) : normalized(w.orientation);
  }
  return out;
}

double distance_to_segment(const Vec3 & p, const Vec3 & a, const Vec3 & b)
{
  const Vec3 ab = minus(b, a);
  const double length2 = dot(ab, ab);
  if (length2 == 0.0) {return position_error(p, a);}
  const double t = std::clamp(dot(minus(p, a), ab) / length2, 0.0, 1.0);
  return position_error(p, {a.x + t * ab.x, a.y + t * ab.y, a.z + t * ab.z});
}

double max_deviation(const std::vector<Vec3> & polyline, const std::vector<Vec3> & points)
{
  if (polyline.empty() || points.empty()) {return std::numeric_limits<double>::quiet_NaN();}
  double worst = 0.0;
  for (const Vec3 & p : points) {
    double nearest = position_error(p, polyline.front());
    for (std::size_t i = 1; i < polyline.size(); ++i) {
      nearest = std::min(nearest, distance_to_segment(p, polyline[i - 1], polyline[i]));
    }
    worst = std::max(worst, nearest);
  }
  return worst;
}

CartesianOutcome move_cartesian(
  CartesianBackend & arm, const CartesianRequest & request, const Settings & settings)
{
  const std::string invalid = check_request(request);
  if (!invalid.empty()) {return failed("goal refused: " + invalid);}

  ControlGuard guard(arm);
  if (settings.use_control) {
    arm.report("control");
    std::string why;
    if (!arm.take_control(why)) {return failed("control refused: " + why);}
    guard.hold();
  }

  Quat start_orientation{0.0, 0.0, 0.0, 1.0};
  const bool keeps_orientation = std::any_of(
    request.waypoints.begin(), request.waypoints.end(),
    [](const Pose & w) {return classify(w) == GoalKind::PositionOnly;});
  if (keeps_orientation) {
    arm.report("measuring start");
    const std::optional<Pose> start = arm.measure();
    if (!start) {return failed("the tcp pose at the start is not available from TF");}
    start_orientation = start->orientation;
  }
  const std::vector<Pose> waypoints = fill_orientation(request.waypoints, start_orientation);

  arm.report("planning");
  std::string why;
  const CartesianPlan plan = arm.plan_cartesian(waypoints, request.eef_step_m, why);
  CartesianOutcome o;
  o.fraction = plan.fraction;
  if (plan.fraction < 0.0) {
    o.message = "no Cartesian path: " + why;
    return o;
  }
  if (!plan.tcp_path.empty()) {
    std::vector<Vec3> polyline{plan.tcp_path.front()};
    for (const Pose & w : waypoints) {
      polyline.push_back(w.position);
    }
    o.max_deviation_m = max_deviation(polyline, plan.tcp_path);
  }
  if (plan.fraction < request.min_fraction) {
    char text[96];
    std::snprintf(text, sizeof(text), "only %.1f%% of the path planned (minimum %.1f%%)",
      plan.fraction * 100.0, request.min_fraction * 100.0);
    o.message = text;
    return o;
  }

  arm.report("executing");
  if (!arm.execute(why)) {
    o.message = "execution failed: " + why;
    return o;
  }

  arm.report("measuring");
  const std::optional<Pose> measured = arm.measure();
  if (!measured) {
    o.message = "moved, but the tcp pose is not available from TF";
    return o;
  }
  o.error = pose_error(waypoints.back(), *measured, true);
  o.success = within(o.error, settings.position_tolerance_m, settings.orientation_tolerance_rad);
  o.message = (o.success ? "reached: " : "outside the tolerance: ") + describe(o);
  return o;
}

}  // namespace braco_mission
