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

// Unit tests of the pick-and-place sequence (pick_and_place.cpp) against a fake arm and a fake
// gripper: no ROS, no MoveIt. The fake arm moves its tcp to what was planned when it executes,
// so every stage measures where the previous one left it.

#include <gtest/gtest.h>

#include <optional>   // cpplint takes it for a C header: it must come first

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include "braco_mission/pick_and_place.hpp"

using braco_mission::above;
using braco_mission::CartesianBackend;
using braco_mission::CartesianPlan;
using braco_mission::check_request;
using braco_mission::GripperBackend;
using braco_mission::MAX_APPROACH_HEIGHT_M;
using braco_mission::pick_and_place;
using braco_mission::PickPlaceOutcome;
using braco_mission::PickPlaceRequest;
using braco_mission::Pose;
using braco_mission::Quat;
using braco_mission::Settings;
using braco_mission::Vec3;
namespace stage = braco_mission::stage;

namespace
{

const Quat DOWN{1.0, 0.0, 0.0, 0.0};   // tcp pointing down
const Quat NONE{0.0, 0.0, 0.0, 0.0};   // position only
const Pose HOME{{0.10, 0.0, 0.25}, DOWN};
const Pose PICK{{0.18, -0.08, 0.03}, DOWN};
const Pose PLACE{{0.18, 0.08, 0.03}, DOWN};
constexpr double HEIGHT = 0.05;

const std::vector<std::string> STAGES = {
  stage::OPEN, stage::PRE_GRASP, stage::APPROACH_PICK, stage::CLOSE, stage::RETREAT_PICK,
  stage::PRE_PLACE, stage::APPROACH_PLACE, stage::RELEASE, stage::RETREAT_PLACE};

PickPlaceRequest request(const Pose & pick = PICK, const Pose & place = PLACE)
{
  PickPlaceRequest r;
  r.pick = pick;
  r.place = place;
  r.approach_height_m = HEIGHT;
  return r;
}

// Every call of the arm and the gripper, in order. `fail` / `fail_nth`: the nth call of that
// name fails.
struct Log
{
  std::vector<std::string> calls;
  std::string fail;
  int fail_nth = 0;

  bool call(const std::string & name)
  {
    calls.push_back(name);
    return !(name == fail && count(name) == fail_nth);
  }
  int count(const std::string & name) const
  {
    int n = 0;
    for (const auto & c : calls) {n += c == name ? 1 : 0;}
    return n;
  }
};

class FakeArm : public CartesianBackend
{
public:
  explicit FakeArm(Log & log)
  : log_(log) {}

  Pose tcp = HOME;
  Vec3 offset{0.0, 0.0, 0.0};   // added to the tcp from the execute numbered `offset_from` on
  int offset_from = 0;
  bool control_granted = true;
  std::vector<std::string> stages;
  std::vector<Pose> targets;     // the end of each plan, in order

  bool take_control(std::string & message) override
  {
    if (!log_.call("take_control") || !control_granted) {
      message = "owned by web";
      return false;
    }
    return true;
  }
  void release_control() override {log_.call("release_control");}
  bool ensure_active(std::string & message) override
  {
    const bool ok = log_.call("ensure_active");
    if (!ok) {message = "arm_controller is not loaded";}
    return ok;
  }
  bool plan(const Pose & goal, bool position_only, std::string & message) override
  {
    Pose target = goal;
    if (position_only) {target.orientation = DOWN;}   // the planner picks one
    targets.push_back(target);
    const bool ok = log_.call("plan");
    if (!ok) {message = "no IK";}
    return ok;
  }
  CartesianPlan plan_cartesian(
    const std::vector<Pose> & waypoints, double, std::string & message) override
  {
    targets.push_back(waypoints.back());
    if (!log_.call("plan_cartesian")) {
      message = "invalid start state";
      return {-1.0, {}};
    }
    return {1.0, {tcp.position, waypoints.back().position}};
  }
  bool execute(std::string & message) override
  {
    if (!log_.call("execute")) {
      message = "aborted";
      return false;
    }
    tcp = targets.back();
    return true;
  }
  std::optional<Pose> measure() override
  {
    log_.call("measure");
    Pose out = tcp;
    if (offset_from > 0 && log_.count("execute") >= offset_from) {
      out.position = {tcp.position.x + offset.x, tcp.position.y + offset.y,
        tcp.position.z + offset.z};
    }
    return out;
  }
  void report(const std::string & s) override {stages.push_back(s);}

private:
  Log & log_;
};

class FakeGripper : public GripperBackend
{
public:
  explicit FakeGripper(Log & log)
  : log_(log) {}

  bool open(std::string & message) override
  {
    const bool ok = log_.call("open");
    if (!ok) {message = "open gripper: aborted";}
    return ok;
  }
  bool close(std::string & message) override
  {
    const bool ok = log_.call("close");
    if (!ok) {message = "set gripper refused: STOP latched";}
    return ok;
  }

private:
  Log & log_;
};

Settings real_arm()
{
  Settings s;
  s.use_control = true;
  return s;
}

bool contains(const std::string & text, const std::string & part)
{
  return text.find(part) != std::string::npos;
}

// Only the arm and gripper commands, not the measurements
std::vector<std::string> commands(const Log & log)
{
  std::vector<std::string> out;
  for (const auto & c : log.calls) {
    if (c != "measure") {out.push_back(c);}
  }
  return out;
}

class PickAndPlaceTest : public ::testing::Test
{
protected:
  Log log_;
  FakeArm arm_{log_};
  FakeGripper gripper_{log_};

  PickPlaceOutcome run(const PickPlaceRequest & r = request(), const Settings & s = Settings())
  {
    return pick_and_place(arm_, gripper_, r, s);
  }
};

// ---- the request ----------------------------------------------------------------------

TEST(PickAndPlaceRequest, AboveMovesUpAndKeepsTheOrientation)
{
  const Pose up = above(PICK, 0.05);
  EXPECT_DOUBLE_EQ(up.position.z, PICK.position.z + 0.05);
  EXPECT_DOUBLE_EQ(up.position.x, PICK.position.x);
  EXPECT_DOUBLE_EQ(up.orientation.x, DOWN.x);
}

TEST(PickAndPlaceRequest, AcceptsPosesAndPositions)
{
  EXPECT_EQ(check_request(request()), "");
  EXPECT_EQ(check_request(request({PICK.position, NONE}, {PLACE.position, NONE})), "");
}

TEST(PickAndPlaceRequest, RefusesWhatIsNotAPose)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_TRUE(contains(check_request(request({{nan, 0.0, 0.0}, DOWN})), "pick pose"));
  EXPECT_TRUE(contains(check_request(request(PICK, {PLACE.position, {0.0, 0.0, 0.0, 2.0}})),
    "place pose"));
}

TEST(PickAndPlaceRequest, RefusesAnApproachHeightOutOfRange)
{
  for (double height : {0.0, -0.01, MAX_APPROACH_HEIGHT_M + 0.01}) {
    PickPlaceRequest r = request();
    r.approach_height_m = height;
    EXPECT_TRUE(contains(check_request(r), "approach height")) << height;
  }
}

// ---- the sequence ---------------------------------------------------------------------

TEST_F(PickAndPlaceTest, RunsEveryStageInOrder)
{
  const PickPlaceOutcome o = run();
  EXPECT_TRUE(o.success) << o.message;
  EXPECT_EQ(o.failed_stage, "");
  EXPECT_EQ(arm_.stages, STAGES);
  // open, pre-grasp, down, close, up, pre-place, down, open, up
  const std::vector<std::string> expected = {
    "open", "plan", "execute", "plan_cartesian", "execute", "close", "plan_cartesian", "execute",
    "plan", "execute", "plan_cartesian", "execute", "open", "plan_cartesian", "execute"};
  EXPECT_EQ(commands(log_), expected);
}

TEST_F(PickAndPlaceTest, GoesStraightUpAndDownAboveEachPose)
{
  run();
  const std::vector<Vec3> expected = {
    above(PICK, HEIGHT).position, PICK.position, above(PICK, HEIGHT).position,
    above(PLACE, HEIGHT).position, PLACE.position, above(PLACE, HEIGHT).position};
  ASSERT_EQ(arm_.targets.size(), expected.size());
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_NEAR(arm_.targets[i].position.x, expected[i].x, 1e-9) << i;
    EXPECT_NEAR(arm_.targets[i].position.y, expected[i].y, 1e-9) << i;
    EXPECT_NEAR(arm_.targets[i].position.z, expected[i].z, 1e-9) << i;
  }
}

TEST_F(PickAndPlaceTest, ReportsThePickAndPlaceErrors)
{
  const PickPlaceOutcome o = run();
  EXPECT_NEAR(o.pick_error.position_m, 0.0, 1e-9);
  EXPECT_NEAR(o.pick_error.orientation_rad, 0.0, 1e-6);
  EXPECT_NEAR(o.place_error.position_m, 0.0, 1e-9);
  EXPECT_TRUE(contains(o.message, "placed: pick error 0.0 mm, place error 0.0 mm")) << o.message;
}

TEST_F(PickAndPlaceTest, PositionOnlyPoseHasNoOrientationError)
{
  const PickPlaceOutcome o = run(request({PICK.position, NONE}, PLACE));
  EXPECT_TRUE(o.success) << o.message;
  EXPECT_TRUE(std::isnan(o.pick_error.orientation_rad));
  EXPECT_FALSE(std::isnan(o.place_error.orientation_rad));
}

TEST_F(PickAndPlaceTest, SimulationTakesNoControl)
{
  run();
  EXPECT_EQ(log_.count("take_control"), 0);
  EXPECT_EQ(log_.count("release_control"), 0);
}

TEST_F(PickAndPlaceTest, RealArmHoldsTheControlForTheWholeSequence)
{
  const PickPlaceOutcome o = run(request(), real_arm());
  EXPECT_TRUE(o.success) << o.message;
  EXPECT_EQ(log_.count("take_control"), 1);
  EXPECT_EQ(log_.count("ensure_active"), 1);
  EXPECT_EQ(log_.count("release_control"), 1);
  EXPECT_EQ(log_.calls.front(), "take_control");
  EXPECT_EQ(log_.calls.back(), "release_control");
  EXPECT_EQ(arm_.stages.front(), stage::CONTROL);
}

// ---- failures -------------------------------------------------------------------------

TEST_F(PickAndPlaceTest, RefusedGoalMovesNothing)
{
  PickPlaceRequest r = request();
  r.approach_height_m = 0.0;
  const PickPlaceOutcome o = run(r, real_arm());
  EXPECT_FALSE(o.success);
  EXPECT_EQ(o.failed_stage, stage::GOAL);
  EXPECT_TRUE(contains(o.message, "goal: refused: approach height")) << o.message;
  EXPECT_TRUE(log_.calls.empty());
}

TEST_F(PickAndPlaceTest, RefusedControlMovesNothing)
{
  arm_.control_granted = false;
  const PickPlaceOutcome o = run(request(), real_arm());
  EXPECT_EQ(o.failed_stage, stage::CONTROL);
  EXPECT_TRUE(contains(o.message, "control: refused: owned by web")) << o.message;
  EXPECT_EQ(log_.calls, std::vector<std::string>{"take_control"});   // nothing to release
}

TEST_F(PickAndPlaceTest, InactiveArmMovesNothingAndReleases)
{
  log_.fail = "ensure_active";
  log_.fail_nth = 1;
  const PickPlaceOutcome o = run(request(), real_arm());
  EXPECT_EQ(o.failed_stage, stage::CONTROL);
  EXPECT_EQ(log_.calls,
    (std::vector<std::string>{"take_control", "ensure_active", "release_control"}));
}

// Each stage fails in turn: the sequence stops at the failing call, nothing moves after it,
// the control is released and the stage is named.
TEST_F(PickAndPlaceTest, AFailingStageStopsTheSequence)
{
  struct Case
  {
    const char * stage;
    const char * call;
    int nth;
  };
  const std::vector<Case> cases = {
    {stage::OPEN, "open", 1},
    {stage::PRE_GRASP, "plan", 1},
    {stage::PRE_GRASP, "execute", 1},
    {stage::APPROACH_PICK, "plan_cartesian", 1},
    {stage::APPROACH_PICK, "execute", 2},
    {stage::CLOSE, "close", 1},
    {stage::RETREAT_PICK, "execute", 3},
    {stage::PRE_PLACE, "plan", 2},
    {stage::APPROACH_PLACE, "execute", 5},
    {stage::RELEASE, "open", 2},
    {stage::RETREAT_PLACE, "plan_cartesian", 4},
  };
  for (const Case & c : cases) {
    Log log;
    log.fail = c.call;
    log.fail_nth = c.nth;
    FakeArm arm(log);
    FakeGripper gripper(log);
    const PickPlaceOutcome o = pick_and_place(arm, gripper, request(), real_arm());
    const std::string where = std::string(c.call) + " #" + std::to_string(c.nth);
    EXPECT_FALSE(o.success) << where;
    EXPECT_EQ(o.failed_stage, c.stage) << where;
    EXPECT_TRUE(contains(o.message, std::string(c.stage) + ": ")) << o.message;
    EXPECT_EQ(arm.stages.back(), c.stage) << where;
    // The failing call, then only the release of the control
    ASSERT_GE(log.calls.size(), 2u) << where;
    EXPECT_EQ(log.calls[log.calls.size() - 2], c.call) << where;
    EXPECT_EQ(log.calls.back(), "release_control") << where;
  }
}

TEST_F(PickAndPlaceTest, GripperRefusalSaysWhy)
{
  log_.fail = "close";
  log_.fail_nth = 1;
  const PickPlaceOutcome o = run();
  EXPECT_EQ(o.message, "close: set gripper refused: STOP latched");
}

TEST_F(PickAndPlaceTest, PickOutsideTheToleranceStopsBeforeClosing)
{
  arm_.offset = {0.0, 0.0, 0.01};   // 1 cm above, from the line down to the pick on
  arm_.offset_from = 2;
  const PickPlaceOutcome o = run();
  EXPECT_FALSE(o.success);
  EXPECT_EQ(o.failed_stage, stage::APPROACH_PICK);
  EXPECT_NEAR(o.pick_error.position_m, 0.01, 1e-9);
  EXPECT_TRUE(std::isnan(o.place_error.position_m));
  EXPECT_EQ(log_.count("close"), 0);
}

TEST_F(PickAndPlaceTest, PlaceOutsideTheToleranceStopsBeforeReleasing)
{
  arm_.offset = {0.01, 0.0, 0.0};
  arm_.offset_from = 5;   // the line down to the place
  const PickPlaceOutcome o = run();
  EXPECT_EQ(o.failed_stage, stage::APPROACH_PLACE);
  EXPECT_NEAR(o.pick_error.position_m, 0.0, 1e-9);
  EXPECT_NEAR(o.place_error.position_m, 0.01, 1e-9);
  EXPECT_EQ(log_.count("open"), 1);   // the gripper still holds the object
}

TEST_F(PickAndPlaceTest, StepsInsideAStageAreNotReported)
{
  run(request(), real_arm());
  for (const auto & s : arm_.stages) {
    EXPECT_TRUE(s == stage::CONTROL || std::find(STAGES.begin(), STAGES.end(), s) != STAGES.end())
      << s;
  }
}

}  // namespace

// ---- cancel and STOP ------------------------------------------------------------------

TEST_F(PickAndPlaceTest, CancelBetweenStagesStartsNothingMore)
{
  // Cancelled during the line down to the pick (control, open, pre_grasp, approach_pick)
  const auto cancelled = [this]() {
      return arm_.stages.size() >= 4 ? std::string("cancelled") : std::string();
    };
  const PickPlaceOutcome o = pick_and_place(arm_, gripper_, request(), real_arm(), cancelled);
  EXPECT_FALSE(o.success);
  EXPECT_EQ(o.failed_stage, stage::CLOSE);
  EXPECT_EQ(o.message, "close: cancelled");
  EXPECT_EQ(log_.count("close"), 0);
  EXPECT_EQ(log_.calls.back(), "release_control");
}

TEST_F(PickAndPlaceTest, InterruptedBeforeTheFirstStageMovesNothing)
{
  const auto stopped = []() {return std::string("STOP latched");};
  const PickPlaceOutcome o = pick_and_place(arm_, gripper_, request(), Settings(), stopped);
  EXPECT_EQ(o.failed_stage, stage::OPEN);
  EXPECT_TRUE(log_.calls.empty());
}

TEST_F(PickAndPlaceTest, StopDuringAMoveGivesTheReasonFirst)
{
  // A STOP while the arm goes down to the pick: the node stops MoveIt, the execution fails
  log_.fail = "execute";
  log_.fail_nth = 2;
  const auto stopped = [this]() {
      return log_.count("execute") >= 2 ? std::string("STOP latched") : std::string();
    };
  const PickPlaceOutcome o = pick_and_place(arm_, gripper_, request(), real_arm(), stopped);
  EXPECT_EQ(o.failed_stage, stage::APPROACH_PICK);
  EXPECT_EQ(o.message, "approach_pick: STOP latched (execution failed: aborted)");
  EXPECT_EQ(log_.count("close"), 0);
  EXPECT_EQ(log_.calls.back(), "release_control");
}
