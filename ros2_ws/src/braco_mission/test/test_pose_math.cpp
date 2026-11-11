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

// Unit tests of the mission's pose math: no ROS.

#include <gtest/gtest.h>

#include <cmath>
#include <limits>

#include "braco_mission/pose_math.hpp"

using braco_mission::classify;
using braco_mission::GoalKind;
using braco_mission::orientation_error;
using braco_mission::Pose;
using braco_mission::pose_error;
using braco_mission::position_error;
using braco_mission::Quat;
using braco_mission::within;

namespace
{

constexpr double NaN = std::numeric_limits<double>::quiet_NaN();
const Quat IDENTITY{0.0, 0.0, 0.0, 1.0};

// Rotation of `deg` about z
Quat about_z(double deg)
{
  const double half = deg * M_PI / 360.0;
  return {0.0, 0.0, std::sin(half), std::cos(half)};
}

}  // namespace

TEST(Classify, UnitQuaternionIsAFullPose)
{
  EXPECT_EQ(classify({{0.1, 0.2, 0.3}, IDENTITY}), GoalKind::FullPose);
}

TEST(Classify, AllZeroOrientationIsPositionOnly)
{
  EXPECT_EQ(classify({{0.1, 0.2, 0.3}, {0.0, 0.0, 0.0, 0.0}}), GoalKind::PositionOnly);
}

TEST(Classify, NearlyUnitQuaternionIsAccepted)
{
  EXPECT_EQ(classify({{0.0, 0.0, 0.2}, {0.0, 0.0, 0.0, 1.0005}}), GoalKind::FullPose);
}

TEST(Classify, QuaternionFarFromUnitIsInvalid)
{
  EXPECT_EQ(classify({{0.0, 0.0, 0.2}, {0.0, 0.0, 0.0, 2.0}}), GoalKind::Invalid);
}

TEST(Classify, NotFiniteIsInvalid)
{
  EXPECT_EQ(classify({{NaN, 0.0, 0.2}, IDENTITY}), GoalKind::Invalid);
  EXPECT_EQ(
    classify({{0.0, 0.0, 0.2}, {0.0, 0.0, std::numeric_limits<double>::infinity(), 1.0}}),
    GoalKind::Invalid);
}

TEST(PositionError, IsTheDistance)
{
  EXPECT_DOUBLE_EQ(position_error({0.0, 0.0, 0.0}, {0.3, 0.4, 0.0}), 0.5);
  EXPECT_DOUBLE_EQ(position_error({0.1, 0.2, 0.3}, {0.1, 0.2, 0.3}), 0.0);
}

TEST(OrientationError, IsTheRotationAngle)
{
  EXPECT_NEAR(orientation_error(IDENTITY, about_z(90.0)), M_PI / 2.0, 1e-12);
  EXPECT_NEAR(orientation_error(about_z(10.0), about_z(-20.0)), 30.0 * M_PI / 180.0, 1e-12);
}

TEST(OrientationError, QAndMinusQAreTheSameRotation)
{
  const Quat q = about_z(40.0);
  EXPECT_NEAR(orientation_error(q, {-q.x, -q.y, -q.z, -q.w}), 0.0, 1e-7);
}

TEST(OrientationError, NormalizesItsInputs)
{
  EXPECT_NEAR(orientation_error({0.0, 0.0, 0.0, 2.0}, IDENTITY), 0.0, 1e-12);
}

TEST(PoseError, PositionOnlyLeavesTheOrientationOut)
{
  const auto e = pose_error({{0.0, 0.0, 0.0}, IDENTITY}, {{0.0, 0.0, 0.01}, about_z(90.0)}, false);
  EXPECT_DOUBLE_EQ(e.position_m, 0.01);
  EXPECT_TRUE(std::isnan(e.orientation_rad));
}

TEST(PoseError, FullPoseComparesBoth)
{
  const auto e = pose_error({{0.0, 0.0, 0.0}, IDENTITY}, {{0.0, 0.0, 0.01}, about_z(90.0)}, true);
  EXPECT_DOUBLE_EQ(e.position_m, 0.01);
  EXPECT_NEAR(e.orientation_rad, M_PI / 2.0, 1e-12);
}

TEST(Within, ChecksBothTolerances)
{
  EXPECT_TRUE(within({0.004, 0.03}, 0.005, 0.035));
  EXPECT_FALSE(within({0.006, 0.03}, 0.005, 0.035));
  EXPECT_FALSE(within({0.004, 0.04}, 0.005, 0.035));
}

TEST(Within, OrientationNotComparedIsNotChecked)
{
  EXPECT_TRUE(within({0.004, NaN}, 0.005, 0.035));
}

TEST(Within, PositionErrorNaNIsNotWithin)
{
  EXPECT_FALSE(within({NaN, 0.0}, 0.005, 0.035));
}
