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

// Unit tests of the steps of a pose goal (mission.cpp) against a fake arm: no ROS, no MoveIt.

#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <optional>
#include <string>
#include <vector>

#include "braco_mission/mission.hpp"

using braco_mission::ArmBackend;
using braco_mission::move_to_pose;
using braco_mission::Pose;
using braco_mission::Settings;

namespace
{

const Pose GOAL{{0.15, 0.0, 0.25}, {0.0, 0.0, 0.0, 1.0}};
const Pose POSITION_GOAL{{0.15, 0.0, 0.25}, {0.0, 0.0, 0.0, 0.0}};

// Records every call; each step succeeds unless told otherwise.
class FakeArm : public ArmBackend
{
public:
  bool control_granted = true;
  bool full_pose_plans = true;
  bool position_plans = true;
  bool executes = true;
  std::optional<Pose> tcp = GOAL;   // what measure() returns

  std::vector<std::string> calls;
  std::vector<std::string> stages;
  std::optional<Pose> planned_goal;

  bool take_control(std::string & message) override
  {
    calls.push_back("take_control");
    if (!control_granted) {message = "owned by web";}
    return control_granted;
  }
  void release_control() override {calls.push_back("release_control");}
  bool plan(const Pose & goal, bool position_only, std::string & message) override
  {
    calls.push_back(position_only ? "plan_position" : "plan_pose");
    planned_goal = goal;
    const bool ok = position_only ? position_plans : full_pose_plans;
    if (!ok) {message = "no IK";}
    return ok;
  }
  bool execute(std::string & message) override
  {
    calls.push_back("execute");
    if (!executes) {message = "aborted";}
    return executes;
  }
  std::optional<Pose> measure() override
  {
    calls.push_back("measure");
    return tcp;
  }
  void report(const std::string & stage) override {stages.push_back(stage);}

  int count(const std::string & call) const
  {
    int n = 0;
    for (const auto & c : calls) {n += c == call ? 1 : 0;}
    return n;
  }
};

Settings real_arm()
{
  Settings s;
  s.use_control = true;
  return s;
}

}  // namespace

TEST(MoveToPose, ReachesAFullPose)
{
  FakeArm arm;
  const auto o = move_to_pose(arm, GOAL, false, Settings{});
  EXPECT_TRUE(o.success);
  EXPECT_FALSE(o.position_only);
  EXPECT_DOUBLE_EQ(o.error.position_m, 0.0);
  EXPECT_NEAR(o.error.orientation_rad, 0.0, 1e-7);
  EXPECT_EQ(arm.calls, (std::vector<std::string>{"plan_pose", "execute", "measure"}));
  EXPECT_EQ(arm.stages, (std::vector<std::string>{"planning", "executing", "measuring"}));
  EXPECT_NE(o.message.find("reached"), std::string::npos);
}

TEST(MoveToPose, SimulationNeverTouchesTheControl)
{
  FakeArm arm;
  arm.executes = false;
  move_to_pose(arm, GOAL, false, Settings{});
  EXPECT_EQ(arm.count("take_control"), 0);
  EXPECT_EQ(arm.count("release_control"), 0);
}

TEST(MoveToPose, RealArmTakesAndReleasesTheControl)
{
  FakeArm arm;
  const auto o = move_to_pose(arm, GOAL, false, real_arm());
  EXPECT_TRUE(o.success);
  EXPECT_EQ(arm.calls.front(), "take_control");
  EXPECT_EQ(arm.calls.back(), "release_control");
  EXPECT_EQ(arm.stages.front(), "control");
}

TEST(MoveToPose, ControlRefusedSendsNothing)
{
  FakeArm arm;
  arm.control_granted = false;
  const auto o = move_to_pose(arm, GOAL, true, real_arm());
  EXPECT_FALSE(o.success);
  EXPECT_EQ(arm.calls, (std::vector<std::string>{"take_control"}));   // nothing planned
  EXPECT_NE(o.message.find("control refused: owned by web"), std::string::npos);
}

TEST(MoveToPose, ControlIsReleasedWhenThePlanFails)
{
  FakeArm arm;
  arm.full_pose_plans = false;
  const auto o = move_to_pose(arm, GOAL, false, real_arm());
  EXPECT_FALSE(o.success);
  EXPECT_EQ(arm.count("execute"), 0);
  EXPECT_EQ(arm.calls.back(), "release_control");
  EXPECT_NE(o.message.find("no plan: no IK"), std::string::npos);
}

TEST(MoveToPose, ControlIsReleasedWhenExecutionFails)
{
  FakeArm arm;
  arm.executes = false;
  const auto o = move_to_pose(arm, GOAL, false, real_arm());
  EXPECT_FALSE(o.success);
  EXPECT_EQ(arm.count("measure"), 0);
  EXPECT_EQ(arm.calls.back(), "release_control");
  EXPECT_NE(o.message.find("execution failed: aborted"), std::string::npos);
}

TEST(MoveToPose, InvalidGoalIsRefusedBeforeTheControl)
{
  FakeArm arm;
  const Pose bad{{std::nan(""), 0.0, 0.25}, {0.0, 0.0, 0.0, 1.0}};
  const auto o = move_to_pose(arm, bad, true, real_arm());
  EXPECT_FALSE(o.success);
  EXPECT_TRUE(arm.calls.empty());
  EXPECT_NE(o.message.find("goal refused"), std::string::npos);
}

TEST(MoveToPose, PositionOnlyGoalPlansThePosition)
{
  FakeArm arm;
  const auto o = move_to_pose(arm, POSITION_GOAL, false, Settings{});
  EXPECT_TRUE(o.success);
  EXPECT_TRUE(o.position_only);
  EXPECT_TRUE(std::isnan(o.error.orientation_rad));
  EXPECT_EQ(arm.count("plan_pose"), 0);
  EXPECT_EQ(arm.count("plan_position"), 1);
}

TEST(MoveToPose, FallsBackToThePositionWhenAsked)
{
  FakeArm arm;
  arm.full_pose_plans = false;
  const auto o = move_to_pose(arm, GOAL, true, Settings{});
  EXPECT_TRUE(o.success);
  EXPECT_TRUE(o.position_only);
  EXPECT_EQ(arm.calls,
    (std::vector<std::string>{"plan_pose", "plan_position", "execute", "measure"}));
  EXPECT_NE(
    std::find(arm.stages.begin(), arm.stages.end(), "planning position only"), arm.stages.end());
}

TEST(MoveToPose, NoFallbackWithoutAsking)
{
  FakeArm arm;
  arm.full_pose_plans = false;
  const auto o = move_to_pose(arm, GOAL, false, Settings{});
  EXPECT_FALSE(o.success);
  EXPECT_EQ(arm.count("plan_position"), 0);
}

TEST(MoveToPose, FallbackThatAlsoFailsReportsIt)
{
  FakeArm arm;
  arm.full_pose_plans = false;
  arm.position_plans = false;
  const auto o = move_to_pose(arm, GOAL, true, Settings{});
  EXPECT_FALSE(o.success);
  EXPECT_TRUE(o.position_only);
  EXPECT_EQ(arm.count("execute"), 0);
}

TEST(MoveToPose, OutsideTheToleranceIsNotASuccess)
{
  FakeArm arm;
  arm.tcp = Pose{{0.15, 0.0, 0.26}, {0.0, 0.0, 0.0, 1.0}};   // 10 mm off
  const auto o = move_to_pose(arm, GOAL, false, Settings{});
  EXPECT_FALSE(o.success);
  EXPECT_NEAR(o.error.position_m, 0.01, 1e-9);
  EXPECT_NE(o.message.find("outside the tolerance: position error 10.0 mm"), std::string::npos);
}

TEST(MoveToPose, OrientationOutsideTheToleranceIsNotASuccess)
{
  FakeArm arm;
  const double half = 5.0 * M_PI / 360.0;   // 5 degrees about z
  arm.tcp = Pose{{0.15, 0.0, 0.25}, {0.0, 0.0, std::sin(half), std::cos(half)}};
  const auto o = move_to_pose(arm, GOAL, false, Settings{});
  EXPECT_FALSE(o.success);
  EXPECT_NEAR(o.error.orientation_rad, 5.0 * M_PI / 180.0, 1e-9);
  EXPECT_NE(o.message.find("orientation error 5.00 deg"), std::string::npos);
}

TEST(MoveToPose, NoTcpPoseIsAFailure)
{
  FakeArm arm;
  arm.tcp.reset();
  const auto o = move_to_pose(arm, GOAL, false, real_arm());
  EXPECT_FALSE(o.success);
  EXPECT_NE(o.message.find("not available from TF"), std::string::npos);
  EXPECT_EQ(arm.calls.back(), "release_control");
}

TEST(MoveToPose, NormalizesTheGoalOrientation)
{
  FakeArm arm;
  const Pose scaled{{0.15, 0.0, 0.25}, {0.0, 0.0, 0.0, 1.0004}};
  move_to_pose(arm, scaled, false, Settings{});
  ASSERT_TRUE(arm.planned_goal.has_value());
  EXPECT_DOUBLE_EQ(arm.planned_goal->orientation.w, 1.0);
}
