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
  const Settings & settings)
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
  // A straight line to `to`, keeping the orientation the tcp has
  const auto line = [&](const Pose & to) {
      CartesianRequest r;
      r.waypoints = {Pose{to.position, {0.0, 0.0, 0.0, 0.0}}};
      r.eef_step_m = request.eef_step_m;
      return move_cartesian(quiet, r, steps);
    };

  arm.report(stage::OPEN);
  if (!gripper.open(why)) {return failed(o, stage::OPEN, why);}

  arm.report(stage::PRE_GRASP);
  const Outcome pre_grasp =
    move_to_pose(quiet, above(request.pick, request.approach_height_m), false, steps);
  if (!pre_grasp.success) {return failed(o, stage::PRE_GRASP, pre_grasp.message);}

  arm.report(stage::APPROACH_PICK);
  const CartesianOutcome down_pick = line(request.pick);
  o.pick_error = error_of(request.pick, down_pick.error);
  if (!down_pick.success) {return failed(o, stage::APPROACH_PICK, down_pick.message);}

  arm.report(stage::CLOSE);
  if (!gripper.close(why)) {return failed(o, stage::CLOSE, why);}

  arm.report(stage::RETREAT_PICK);
  const CartesianOutcome up_pick = line(above(request.pick, request.approach_height_m));
  if (!up_pick.success) {return failed(o, stage::RETREAT_PICK, up_pick.message);}

  arm.report(stage::PRE_PLACE);
  const Outcome pre_place =
    move_to_pose(quiet, above(request.place, request.approach_height_m), false, steps);
  if (!pre_place.success) {return failed(o, stage::PRE_PLACE, pre_place.message);}

  arm.report(stage::APPROACH_PLACE);
  const CartesianOutcome down_place = line(request.place);
  o.place_error = error_of(request.place, down_place.error);
  if (!down_place.success) {return failed(o, stage::APPROACH_PLACE, down_place.message);}

  arm.report(stage::RELEASE);
  if (!gripper.open(why)) {return failed(o, stage::RELEASE, why);}

  arm.report(stage::RETREAT_PLACE);
  const CartesianOutcome up_place = line(above(request.place, request.approach_height_m));
  if (!up_place.success) {return failed(o, stage::RETREAT_PLACE, up_place.message);}

  o.success = true;
  o.message = "placed: pick error " + millimetres(o.pick_error) + ", place error " +
    millimetres(o.place_error);
  return o;
}

}  // namespace braco_mission
