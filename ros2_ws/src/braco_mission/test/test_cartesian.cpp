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

// Unit tests of the straight-line move (cartesian.cpp) against a fake arm: no ROS, no MoveIt.

#include <gtest/gtest.h>

#include <optional>   // cpplint takes it for a C header: it must come first

#include <cmath>
#include <string>
#include <vector>

#include "braco_mission/cartesian.hpp"

using braco_mission::CartesianBackend;
using braco_mission::CartesianPlan;
using braco_mission::CartesianRequest;
using braco_mission::check_request;
using braco_mission::distance_to_segment;
using braco_mission::fill_orientation;
using braco_mission::max_deviation;
using braco_mission::move_cartesian;
using braco_mission::Pose;
using braco_mission::Quat;
using braco_mission::Settings;
using braco_mission::Vec3;

namespace
{

const Quat DOWN{1.0, 0.0, 0.0, 0.0};           // tcp pointing down
const Pose START{{0.15, -0.05, 0.10}, DOWN};
const Pose END{{0.15, 0.05, 0.10}, DOWN};      // 10 cm along y
const Pose END_POSITION_ONLY{{0.15, 0.05, 0.10}, {0.0, 0.0, 0.0, 0.0}};

CartesianRequest line(const Pose & end = END)
{
  CartesianRequest r;
  r.waypoints = {end};
  return r;
}

// A straight tcp path from START to END, one point per centimetre.
std::vector<Vec3> straight_path()
{
  std::vector<Vec3> path;
  for (int i = 0; i <= 10; ++i) {
    path.push_back({0.15, -0.05 + 0.01 * i, 0.10});
  }
  return path;
}

// Records every call; each step succeeds unless told otherwise.
class FakeArm : public CartesianBackend
{
public:
  bool control_granted = true;
  CartesianPlan planned{1.0, straight_path()};
  bool executes = true;
  std::optional<Pose> tcp = END;   // what measure() returns

  std::vector<std::string> calls;
  std::vector<std::string> stages;
  std::vector<Pose> planned_waypoints;
  double planned_eef_step = 0.0;

  bool take_control(std::string & message) override
  {
    calls.push_back("take_control");
    if (!control_granted) {message = "owned by web";}
    return control_granted;
  }
  void release_control() override {calls.push_back("release_control");}
  bool plan(const Pose &, bool, std::string &) override
  {
    calls.push_back("plan");
    return false;
  }
  CartesianPlan plan_cartesian(
    const std::vector<Pose> & waypoints, double eef_step, std::string & message) override
  {
    calls.push_back("plan_cartesian");
    planned_waypoints = waypoints;
    planned_eef_step = eef_step;
    if (planned.fraction < 0.0) {message = "invalid start state";}
    return planned;
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

bool contains(const std::string & text, const std::string & part)
{
  return text.find(part) != std::string::npos;
}

}  // namespace

// ---- check_request ---------------------------------------------------------------------

TEST(CheckRequest, AcceptsALine)
{
  EXPECT_EQ(check_request(line()), "");
}

TEST(CheckRequest, AcceptsAPositionOnlyWaypoint)
{
  EXPECT_EQ(check_request(line(END_POSITION_ONLY)), "");
}

TEST(CheckRequest, RefusesNoWaypoints)
{
  EXPECT_EQ(check_request(CartesianRequest{}), "no waypoints");
}

TEST(CheckRequest, RefusesAWaypointThatIsNotAPose)
{
  const Pose nan_pose{{std::nan(""), 0.0, 0.1}, DOWN};
  const Pose not_rotation{{0.15, 0.0, 0.1}, {0.0, 0.0, 0.0, 2.0}};
  EXPECT_TRUE(contains(check_request(line(nan_pose)), "waypoint 0 is not finite"));
  EXPECT_TRUE(contains(check_request(line(not_rotation)), "waypoint 0 is not finite"));
}

TEST(CheckRequest, RefusesTwoWaypointsAtTheSamePlace)
{
  CartesianRequest r = line();
  r.waypoints.push_back(END);
  EXPECT_TRUE(contains(check_request(r), "waypoint 1 is at the same place as waypoint 0"));
}

TEST(CheckRequest, RefusesAnEefStepOutOfRange)
{
  for (const double step : {0.0, -0.001, 0.051, std::nan("")}) {
    CartesianRequest r = line();
    r.eef_step_m = step;
    EXPECT_TRUE(contains(check_request(r), "eef_step")) << step;
  }
  CartesianRequest largest = line();
  largest.eef_step_m = braco_mission::MAX_EEF_STEP_M;
  EXPECT_EQ(check_request(largest), "");
}

TEST(CheckRequest, RefusesAMinFractionOutOfRange)
{
  for (const double fraction : {0.0, 1.01, std::nan("")}) {
    CartesianRequest r = line();
    r.min_fraction = fraction;
    EXPECT_TRUE(contains(check_request(r), "min_fraction")) << fraction;
  }
}

// ---- fill_orientation ------------------------------------------------------------------

TEST(FillOrientation, PositionOnlyTakesTheStartOrientation)
{
  const auto out = fill_orientation({END_POSITION_ONLY}, {0.0, 0.0, 0.0, 2.0});
  EXPECT_DOUBLE_EQ(out[0].orientation.w, 1.0);   // and normalized
  EXPECT_DOUBLE_EQ(out[0].position.y, 0.05);
}

TEST(FillOrientation, KeepsAGivenOrientationNormalized)
{
  const Pose scaled{{0.15, 0.05, 0.10}, {1.0004, 0.0, 0.0, 0.0}};
  const auto out = fill_orientation({scaled}, {0.0, 0.0, 0.0, 1.0});
  EXPECT_DOUBLE_EQ(out[0].orientation.x, 1.0);
  EXPECT_DOUBLE_EQ(out[0].orientation.w, 0.0);
}

// ---- distance_to_segment / max_deviation -----------------------------------------------

TEST(DistanceToSegment, OnThePerpendicularBesideAndBeyondTheEnds)
{
  const Vec3 a{0.0, 0.0, 0.0};
  const Vec3 b{0.1, 0.0, 0.0};
  EXPECT_DOUBLE_EQ(distance_to_segment({0.05, 0.0, 0.0}, a, b), 0.0);
  EXPECT_DOUBLE_EQ(distance_to_segment({0.05, 0.002, 0.0}, a, b), 0.002);
  EXPECT_DOUBLE_EQ(distance_to_segment({0.13, 0.0, 0.04}, a, b), 0.05);   // past b
  EXPECT_DOUBLE_EQ(distance_to_segment({-0.03, 0.04, 0.0}, a, b), 0.05);  // before a
  EXPECT_DOUBLE_EQ(distance_to_segment({0.03, 0.04, 0.0}, a, a), 0.05);   // a == b
}

TEST(MaxDeviation, StraightPathIsZero)
{
  EXPECT_NEAR(max_deviation({START.position, END.position}, straight_path()), 0.0, 1e-12);
}

TEST(MaxDeviation, ReportsTheWorstPoint)
{
  std::vector<Vec3> path = straight_path();
  path[3].z += 0.001;
  path[6].x -= 0.002;
  EXPECT_NEAR(max_deviation({START.position, END.position}, path), 0.002, 1e-12);
}

TEST(MaxDeviation, TakesTheNearestSegmentOfAPolyline)
{
  // An L: a corner point is on the second segment, far from the first one
  const std::vector<Vec3> polyline{{0.0, 0.0, 0.0}, {0.1, 0.0, 0.0}, {0.1, 0.1, 0.0}};
  EXPECT_NEAR(max_deviation(polyline, {{0.1, 0.05, 0.0}, {0.05, 0.0, 0.0}}), 0.0, 1e-12);
  EXPECT_NEAR(max_deviation({{0.0, 0.0, 0.0}}, {{0.0, 0.0, 0.003}}), 0.003, 1e-12);
}

TEST(MaxDeviation, NothingToCompareIsNaN)
{
  EXPECT_TRUE(std::isnan(max_deviation({}, straight_path())));
  EXPECT_TRUE(std::isnan(max_deviation({START.position}, {})));
}

// ---- move_cartesian --------------------------------------------------------------------

TEST(MoveCartesian, RunsAStraightLine)
{
  FakeArm arm;
  const auto o = move_cartesian(arm, line(), Settings{});
  EXPECT_TRUE(o.success);
  EXPECT_DOUBLE_EQ(o.fraction, 1.0);
  EXPECT_NEAR(o.max_deviation_m, 0.0, 1e-12);
  EXPECT_NEAR(o.error.position_m, 0.0, 1e-12);
  EXPECT_NEAR(o.error.orientation_rad, 0.0, 1e-7);
  EXPECT_EQ(arm.calls, (std::vector<std::string>{"plan_cartesian", "execute", "measure"}));
  EXPECT_EQ(arm.stages, (std::vector<std::string>{"planning", "executing", "measuring"}));
  EXPECT_DOUBLE_EQ(arm.planned_eef_step, 0.005);
  EXPECT_TRUE(contains(o.message, "reached: "));
  EXPECT_TRUE(contains(o.message, "line deviation 0.0 mm"));
}

TEST(MoveCartesian, InvalidRequestIsRefusedBeforeTheControl)
{
  FakeArm arm;
  const auto o = move_cartesian(arm, CartesianRequest{}, real_arm());
  EXPECT_FALSE(o.success);
  EXPECT_TRUE(arm.calls.empty());
  EXPECT_EQ(o.message, "goal refused: no waypoints");
}

TEST(MoveCartesian, RealArmTakesAndReleasesTheControl)
{
  FakeArm arm;
  const auto o = move_cartesian(arm, line(), real_arm());
  EXPECT_TRUE(o.success);
  EXPECT_EQ(arm.calls.front(), "take_control");
  EXPECT_EQ(arm.calls.back(), "release_control");
  EXPECT_EQ(arm.stages.front(), "control");
}

TEST(MoveCartesian, SimulationNeverTouchesTheControl)
{
  FakeArm arm;
  arm.executes = false;
  move_cartesian(arm, line(), Settings{});
  EXPECT_EQ(arm.count("take_control"), 0);
  EXPECT_EQ(arm.count("release_control"), 0);
}

TEST(MoveCartesian, ControlRefusedPlansNothing)
{
  FakeArm arm;
  arm.control_granted = false;
  const auto o = move_cartesian(arm, line(), real_arm());
  EXPECT_FALSE(o.success);
  EXPECT_EQ(arm.calls, (std::vector<std::string>{"take_control"}));
  EXPECT_EQ(o.message, "control refused: owned by web");
}

TEST(MoveCartesian, PositionOnlyWaypointKeepsTheStartOrientation)
{
  FakeArm arm;
  const auto o = move_cartesian(arm, line(END_POSITION_ONLY), Settings{});
  EXPECT_TRUE(o.success);
  EXPECT_EQ(arm.calls,
    (std::vector<std::string>{"measure", "plan_cartesian", "execute", "measure"}));
  EXPECT_EQ(arm.stages.front(), "measuring start");
  ASSERT_EQ(arm.planned_waypoints.size(), 1u);
  EXPECT_DOUBLE_EQ(arm.planned_waypoints[0].orientation.x, 1.0);   // DOWN, from measure()
}

TEST(MoveCartesian, NoStartPoseIsAFailure)
{
  FakeArm arm;
  arm.tcp.reset();
  const auto o = move_cartesian(arm, line(END_POSITION_ONLY), real_arm());
  EXPECT_FALSE(o.success);
  EXPECT_EQ(arm.count("plan_cartesian"), 0);
  EXPECT_TRUE(contains(o.message, "at the start is not available"));
  EXPECT_EQ(arm.calls.back(), "release_control");
}

TEST(MoveCartesian, PlanningErrorExecutesNothing)
{
  FakeArm arm;
  arm.planned = CartesianPlan{-1.0, {}};
  const auto o = move_cartesian(arm, line(), real_arm());
  EXPECT_FALSE(o.success);
  EXPECT_DOUBLE_EQ(o.fraction, -1.0);
  EXPECT_TRUE(std::isnan(o.max_deviation_m));
  EXPECT_EQ(arm.count("execute"), 0);
  EXPECT_EQ(o.message, "no Cartesian path: invalid start state");
  EXPECT_EQ(arm.calls.back(), "release_control");
}

TEST(MoveCartesian, PartialPathExecutesNothing)
{
  FakeArm arm;
  std::vector<Vec3> half = straight_path();
  half.resize(7);
  arm.planned = CartesianPlan{0.6, half};
  const auto o = move_cartesian(arm, line(), real_arm());
  EXPECT_FALSE(o.success);
  EXPECT_DOUBLE_EQ(o.fraction, 0.6);
  EXPECT_NEAR(o.max_deviation_m, 0.0, 1e-12);   // what was planned is still reported
  EXPECT_EQ(arm.count("execute"), 0);
  EXPECT_EQ(o.message, "only 60.0% of the path planned (minimum 100.0%)");
  EXPECT_EQ(arm.calls.back(), "release_control");
}

TEST(MoveCartesian, LowerMinFractionAcceptsAPartialPath)
{
  FakeArm arm;
  arm.planned.fraction = 0.6;
  CartesianRequest r = line();
  r.min_fraction = 0.5;
  const auto o = move_cartesian(arm, r, Settings{});
  EXPECT_EQ(arm.count("execute"), 1);
  EXPECT_TRUE(o.success);
}

TEST(MoveCartesian, EmptyTcpPathLeavesTheDeviationUnknown)
{
  FakeArm arm;
  arm.planned.tcp_path.clear();
  const auto o = move_cartesian(arm, line(), Settings{});
  EXPECT_TRUE(o.success);
  EXPECT_TRUE(std::isnan(o.max_deviation_m));
}

TEST(MoveCartesian, ExecutionFailureIsReported)
{
  FakeArm arm;
  arm.executes = false;
  const auto o = move_cartesian(arm, line(), real_arm());
  EXPECT_FALSE(o.success);
  EXPECT_EQ(arm.count("measure"), 0);
  EXPECT_EQ(o.message, "execution failed: aborted");
  EXPECT_EQ(arm.calls.back(), "release_control");
}

TEST(MoveCartesian, NoTcpPoseAfterTheMoveIsAFailure)
{
  FakeArm arm;
  arm.tcp.reset();
  const auto o = move_cartesian(arm, line(), Settings{});
  EXPECT_FALSE(o.success);
  EXPECT_EQ(o.message, "moved, but the tcp pose is not available from TF");
}

TEST(MoveCartesian, OutsideTheToleranceIsNotASuccess)
{
  FakeArm arm;
  arm.tcp = Pose{{0.15, 0.05, 0.11}, DOWN};   // 10 mm above the end
  const auto o = move_cartesian(arm, line(), Settings{});
  EXPECT_FALSE(o.success);
  EXPECT_NEAR(o.error.position_m, 0.01, 1e-9);
  EXPECT_TRUE(contains(o.message, "outside the tolerance: position error 10.0 mm"));
}

TEST(MoveCartesian, ComparesTheLastWaypoint)
{
  FakeArm arm;
  CartesianRequest r;
  r.waypoints = {Pose{{0.15, 0.0, 0.10}, DOWN}, END};
  const auto o = move_cartesian(arm, r, Settings{});
  EXPECT_TRUE(o.success);
  EXPECT_EQ(arm.planned_waypoints.size(), 2u);
  EXPECT_EQ(arm.count("plan"), 0);   // never the point-to-point planner
}
