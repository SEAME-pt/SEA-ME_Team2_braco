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

// speed_for_velocity(): the servo speed write() sends for arm_controller's velocity commands.
// No ROS: the function alone.

#include <gtest/gtest.h>

#include <cmath>

#include "braco_hardware/braco_system.hpp"

using braco_hardware::JointValues;
using braco_hardware::speed_for_velocity;

namespace
{

// The URDF's velocity limits, rad/s at speed 100
constexpr JointValues MAX = {2.64, 2.44, 2.72, 2.69, 2.65, 2.60};
constexpr uint8_t MIN_SPEED = 5;

}  // namespace

// Stopped (start and end of a trajectory): min_speed, so the arm never crawls to the target.
TEST(SpeedForVelocity, StoppedGivesMinSpeed)
{
  EXPECT_EQ(speed_for_velocity({0, 0, 0, 0, 0, 0}, MAX, MIN_SPEED), MIN_SPEED);
}

// No velocity written yet: min_speed.
TEST(SpeedForVelocity, NanGivesMinSpeed)
{
  EXPECT_EQ(speed_for_velocity({0.5, NAN, 0, 0, 0, 0}, MAX, MIN_SPEED), MIN_SPEED);
}

// One speed for all six joints: the fastest one, relative to its own max, sets it.
// joint2: 0.5 / 2.44 x 100 = 20.5, x 1.5 = 30.7, rounded up: 31. joint1 alone would give 12.
TEST(SpeedForVelocity, FastestJointWinsRoundedUp)
{
  EXPECT_EQ(speed_for_velocity({0.2, 0.5, 0, 0, 0, 0}, MAX, MIN_SPEED), 31);
}

// Moving backwards counts the same as forwards.
TEST(SpeedForVelocity, NegativeVelocityCountsItsSize)
{
  EXPECT_EQ(speed_for_velocity({0, -0.5, 0, 0, 0, 0}, MAX, MIN_SPEED), 31);
}

// Faster than the servo can go: 100, never more.
TEST(SpeedForVelocity, CappedAt100)
{
  EXPECT_EQ(speed_for_velocity({0, 0, 0, 0, 0, 2.6}, MAX, MIN_SPEED), 100);
}

// A slow joint: the margin and the floor, never below min_speed.
TEST(SpeedForVelocity, SlowGivesAtLeastMinSpeed)
{
  EXPECT_EQ(speed_for_velocity({0.01, 0, 0, 0, 0, 0}, MAX, MIN_SPEED), MIN_SPEED);
  EXPECT_EQ(speed_for_velocity({0.01, 0, 0, 0, 0, 0}, MAX, 1), 1);
}
