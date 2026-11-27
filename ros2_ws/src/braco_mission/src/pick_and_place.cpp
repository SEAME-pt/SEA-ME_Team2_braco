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

#include "braco_mission/pick_and_place.hpp"

#include <optional>   // cpplint takes it for a C header: it must come first

#include <cmath>
#include <cstdio>
#include <functional>
#include <string>
#include <utility>
#include <vector>

#include "braco_mission/control_guard.hpp"

namespace braco_mission
{

namespace
{

// The arm as the steps of a stage see it: every call goes to `arm`, but their reports
// (planning, executing, ...) are dropped, so the feedback stays the stage.
class StageArm : public CartesianBackend
{
public:
  explicit StageArm(CartesianBackend & arm)
  : arm_(arm) {}

  bool take_control(std::string & message) override {return arm_.take_control(message);}
  void release_control() override {arm_.release_control();}
  bool ensure_active(std::string & message) override {return arm_.ensure_active(message);}
  bool plan(const Pose & goal, bool position_only, std::string & message) override
  {
    return arm_.plan(goal, position_only, message);
  }
  bool execute(std::string & message) override {return arm_.execute(message);}
  std::optional<Pose> measure() override {return arm_.measure();}
  void report(const std::string &) override {}
  CartesianPlan plan_cartesian(
    const std::vector<Pose> & waypoints, double eef_step, std::string & message) override
  {
    return arm_.plan_cartesian(waypoints, eef_step, message);
  }

private:
  CartesianBackend & arm_;
};

PickPlaceOutcome failed(PickPlaceOutcome o, const char * at, const std::string & why)
{
  o.success = false;
  o.failed_stage = at;
  o.message = std::string(at) + ": " + why;
  return o;
}

std::string millimetres(const PoseError & e)
{
  char text[32];
  std::snprintf(text, sizeof(text), "%.1f mm", e.position_m * 1000.0);
  return text;
}

}  // namespace

std::string check_request(const PickPlaceRequest & request)
{
  if (classify(request.pick) == GoalKind::Invalid) {
    return "the pick pose is not finite, or its orientation is not a rotation";
  }
  if (classify(request.place) == GoalKind::Invalid) {
    return "the place pose is not finite, or its orientation is not a rotation";
  }
  if (!(request.approach_height_m > 0.0 && request.approach_height_m <= MAX_APPROACH_HEIGHT_M)) {
    char text[64];
    std::snprintf(text, sizeof(text), "approach height must be in (0, %.2f] m",
      MAX_APPROACH_HEIGHT_M);
    return text;
  }
  return "";
}

Pose above(const Pose & pose, double height)
{
  Pose out = pose;
  out.position.z += height;
  return out;
}

PickPlaceOutcome pick_and_place(
  CartesianBackend & arm, GripperBackend & gripper, const PickPlaceRequest & request,
  const Settings & settings, const Interrupted & interrupted)
{
  PickPlaceOutcome o;
  const std::string invalid = check_request(request);
  if (!invalid.empty()) {return failed(o, stage::GOAL, "refused: " + invalid);}

  // The control is held for the whole sequence: the steps run without taking it again
  ControlGuard guard(arm);
  std::string why;
  if (settings.use_control) {
    arm.report(stage::CONTROL);
    if (!arm.take_control(why)) {return failed(o, stage::CONTROL, "refused: " + why);}
    guard.hold();
    if (!arm.ensure_active(why)) {return failed(o, stage::CONTROL, "not active: " + why);}
  }
  Settings steps = settings;
  steps.use_control = false;
  StageArm quiet(arm);

  // A position-only pose has no orientation to compare
  const auto error_of = [](const Pose & goal, const PoseError & e) {
      PoseError out = e;
      if (classify(goal) == GoalKind::PositionOnly) {out.orientation_rad = std::nan("");}
      return out;
    };
  // Point to point to `to`
  const auto pose = [&](const Pose & to, std::string & message) {
      const Outcome m = move_to_pose(quiet, to, false, steps);
      message = m.message;
      return m.success;
    };
  // A straight line to `to`, keeping the orientation the tcp has; `error` gets its pose error
  const auto line = [&](const Pose & to, std::string & message, PoseError * error) {
      CartesianRequest r;
      r.waypoints = {Pose{to.position, {0.0, 0.0, 0.0, 0.0}}};
      r.eef_step_m = request.eef_step_m;
      const CartesianOutcome m = move_cartesian(quiet, r, steps);
      if (error) {*error = error_of(to, m.error);}
      message = m.message;
      return m.success;
    };
  const Pose over_pick = above(request.pick, request.approach_height_m);
  const Pose over_place = above(request.place, request.approach_height_m);

  struct Stage
  {
    const char * name;
    std::function<bool(std::string &)> run;   // false: the message says why
  };
  const std::vector<Stage> sequence = {
    {stage::OPEN, [&](std::string & m) {return gripper.open(m);}},
    {stage::PRE_GRASP, [&](std::string & m) {return pose(over_pick, m);}},
    {stage::APPROACH_PICK, [&](std::string & m) {return line(request.pick, m, &o.pick_error);}},
    {stage::CLOSE, [&](std::string & m) {return gripper.close(m);}},
    {stage::RETREAT_PICK, [&](std::string & m) {return line(over_pick, m, nullptr);}},
    {stage::PRE_PLACE, [&](std::string & m) {return pose(over_place, m);}},
    {stage::APPROACH_PLACE, [&](std::string & m) {return line(request.place, m, &o.place_error);}},
    {stage::RELEASE, [&](std::string & m) {return gripper.open(m);}},
    {stage::RETREAT_PLACE, [&](std::string & m) {return line(over_place, m, nullptr);}},
  };
  const auto reason = [&interrupted]() {return interrupted ? interrupted() : std::string();};

  for (const Stage & s : sequence) {
    const std::string stop = reason();
    if (!stop.empty()) {return failed(o, s.name, stop);}
    arm.report(s.name);
    if (!s.run(why)) {
      std::string message = reason();   // a cancel or a STOP stopped the move
      if (message.empty()) {
        message = why;
      } else {
        message.append(" (").append(why).append(")");
      }
      return failed(o, s.name, message);
    }
  }

  o.success = true;
  o.message = "placed: pick error " + millimetres(o.pick_error) + ", place error " +
    millimetres(o.place_error);
  return o;
}

}  // namespace braco_mission
