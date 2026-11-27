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

#include <gtest/gtest.h>

#include <optional>   // cpplint takes it for a C header: it must come first

#include <map>
#include <set>

#include "teleop_logic.hpp"

namespace tl = mycobot::teleop;

TEST(StickRatio, DeadzoneIsZero)
{
  EXPECT_DOUBLE_EQ(tl::stickRatio(0), 0.0);
  EXPECT_DOUBLE_EQ(tl::stickRatio(tl::AXIS_DEADZONE), 0.0);
  EXPECT_DOUBLE_EQ(tl::stickRatio(-tl::AXIS_DEADZONE), 0.0);
}

TEST(StickRatio, FullDeflectionIsOne)
{
  EXPECT_DOUBLE_EQ(tl::stickRatio(32767), 1.0);
  EXPECT_DOUBLE_EQ(tl::stickRatio(-32767), 1.0);
}

// ---- Open-loop jog target -------------------------------------------------------

// Blue mode, full stick: speed 15 (22.8 deg/s), lead 5. Bound: 5 + 22.8 * 0.2 = 9.56 deg.
constexpr int SPEED = 15;
constexpr double LEAD = 5.0;
constexpr double FAR = 100.0;   // degrees to the limit: never the bound here
constexpr double TICK = 0.02;
const double VELOCITY = SPEED * tl::DEG_PER_S_PER_SPEED;
const double REACH = LEAD + VELOCITY * tl::FEEDBACK_LAG_S;

// The first target jumps BREAKAWAY_DEG ahead, to break static friction.
TEST(AdvanceJogTarget, StartsWithTheBreakawayStep)
{
  EXPECT_DOUBLE_EQ(
    tl::advanceJogTarget(std::nullopt, 10.0, 1, SPEED, LEAD, FAR, 0.0), 10.0 + tl::BREAKAWAY_DEG);
  EXPECT_DOUBLE_EQ(
    tl::advanceJogTarget(std::nullopt, 10.0, -1, SPEED, LEAD, FAR, 0.0), 10.0 - tl::BREAKAWAY_DEG);
}

// The point: a stale measured angle (not moving) does not stall the target. It keeps
// moving at the jog velocity, the same step every tick, until the bound.
TEST(AdvanceJogTarget, AdvancesAtTheJogVelocityWhileFeedbackIsStale)
{
  std::optional<double> target;
  double previous = 0.0;
  for (int i = 0; i < 10; ++i) {
    target = tl::advanceJogTarget(target, 0.0, 1, SPEED, LEAD, FAR, TICK);
    if (i > 0) {EXPECT_NEAR(*target - previous, VELOCITY * TICK, 1e-9) << "tick " << i;}
    previous = *target;
  }
  EXPECT_LE(*target, REACH);
}

// Never more than lead + velocity * FEEDBACK_LAG_S ahead of the measured angle (about `lead`
// ahead of the real arm), and never behind it.
TEST(AdvanceJogTarget, StaysBetweenTheMeasuredAngleAndTheBound)
{
  EXPECT_DOUBLE_EQ(tl::advanceJogTarget(50.0, 0.0, 1, SPEED, LEAD, FAR, TICK), REACH);
  EXPECT_DOUBLE_EQ(tl::advanceJogTarget(-50.0, 0.0, -1, SPEED, LEAD, FAR, TICK), -REACH);
  EXPECT_DOUBLE_EQ(tl::advanceJogTarget(-3.0, 0.0, 1, SPEED, LEAD, FAR, 0.0), 0.0);
}

// Near the limit the remaining degrees win, also for the breakaway step.
TEST(AdvanceJogTarget, NeverPastTheLimit)
{
  EXPECT_DOUBLE_EQ(tl::advanceJogTarget(std::nullopt, 0.0, 1, SPEED, LEAD, 1.5, 0.0), 1.5);
  EXPECT_DOUBLE_EQ(tl::advanceJogTarget(1.4, 0.0, 1, SPEED, LEAD, 1.5, TICK), 1.5);
}

TEST(ShapeNearLimit, FarFromLimitKeepsLead)
{
  const mycobot::JointLimit lim{-100.0, 100.0};
  const auto s = tl::shapeNearLimit(lim, 0.0, 1, 30.0, 10.0);
  ASSERT_TRUE(s);
  EXPECT_DOUBLE_EQ(s->target, 10.0);
  EXPECT_EQ(s->speed, 30);
}

TEST(ShapeNearLimit, NeverStepsPastLimit)
{
  const mycobot::JointLimit lim{-100.0, 100.0};
  const auto s = tl::shapeNearLimit(lim, 98.0, 1, 30.0, 25.0);
  ASSERT_TRUE(s);
  EXPECT_LE(s->target, 100.0);
  EXPECT_LT(s->speed, 30);
}

TEST(ShapeNearLimit, AtLimitGivesNothing)
{
  const mycobot::JointLimit lim{-100.0, 100.0};
  EXPECT_FALSE(tl::shapeNearLimit(lim, 99.9, 1, 30.0, 10.0));
  // Moving away from the limit is still allowed
  EXPECT_TRUE(tl::shapeNearLimit(lim, 99.9, -1, 30.0, 10.0));
}

TEST(WantedMotion, LeftStickMovesJoint1Inverted)
{
  const std::map<int, int> axes = {{0, 32767}};
  const auto w = tl::wantedMotion(axes, {}, tl::SPEED_MODES[tl::START_MODE], {});
  ASSERT_EQ(w.size(), 1u);
  ASSERT_TRUE(w.count(1));
  EXPECT_EQ(w.at(1).direction, -1);
  EXPECT_DOUBLE_EQ(w.at(1).speed, tl::SPEED_MODES[tl::START_MODE].maxSpeed);
}

TEST(WantedMotion, TriggersAndBlockedAxesDoNothing)
{
  const std::map<int, int> axes = {{1, 32767}};
  const std::set<int> blocked = {1};
  EXPECT_TRUE(tl::wantedMotion(axes, {}, tl::SPEED_MODES[0], blocked).empty());
  EXPECT_TRUE(tl::isTriggerAxis(4));
  EXPECT_TRUE(tl::isTriggerAxis(5));
}

TEST(WantedMotion, BumpersAreExclusive)
{
  const std::map<int, int> axes = {{0, 32767}};
  const std::map<int, int> buttons = {{tl::BUTTON_R1, 1}};
  const auto w = tl::wantedMotion(axes, buttons, tl::SPEED_MODES[0], {});
  ASSERT_EQ(w.size(), 1u);
  EXPECT_EQ(w.at(6).direction, 1);
}

TEST(HomeStep, BoundedAndSynchronised)
{
  const mycobot::Angles angles = {60.0, -30.0, 0.0, 15.0, 0.0, 0.0};
  const auto t = tl::homeStep(angles);
  ASSERT_TRUE(t);
  // Largest joint moves HOME_LEAD_DEG, the others the same fraction
  EXPECT_DOUBLE_EQ((*t)[0], 45.0);
  EXPECT_DOUBLE_EQ((*t)[1], -22.5);
  EXPECT_NEAR((*t)[3], 11.3, 0.05);
}

TEST(HomeStep, DoneWithinTolerance)
{
  const mycobot::Angles angles = {0.5, -0.5, 0.0, 0.9, 0.0, 0.0};
  EXPECT_FALSE(tl::homeStep(angles));
}

TEST(ButtonHold, FiresOnceAfterTheHoldTime)
{
  tl::ButtonHold hold(1.0);
  EXPECT_FALSE(hold.update(true, 10.0));
  EXPECT_FALSE(hold.update(true, 10.9));
  EXPECT_TRUE(hold.update(true, 11.0));
  EXPECT_FALSE(hold.update(true, 11.5));   // still the same press
  EXPECT_FALSE(hold.update(true, 15.0));
}

TEST(ButtonHold, ShortPressesNeverFire)
{
  tl::ButtonHold hold(1.0);
  EXPECT_FALSE(hold.update(true, 0.0));
  EXPECT_FALSE(hold.update(false, 0.6));
  EXPECT_FALSE(hold.update(true, 0.8));    // the timer starts again
  EXPECT_FALSE(hold.update(true, 1.5));
  EXPECT_TRUE(hold.update(true, 1.8));
}

TEST(ButtonHold, ReleaseRearms)
{
  tl::ButtonHold hold(1.0);
  hold.update(true, 0.0);
  EXPECT_TRUE(hold.update(true, 1.0));
  EXPECT_FALSE(hold.update(false, 1.2));
  EXPECT_FALSE(hold.update(true, 2.0));
  EXPECT_TRUE(hold.update(true, 3.0));
}
