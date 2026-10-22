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

// Unit tests of the control owner and of the auto-rest decision: no ROS, no arm.

#include <gtest/gtest.h>

#include <chrono>

#include "braco_driver/auto_rest.hpp"
#include "braco_driver/control.hpp"

using namespace std::chrono_literals;
using braco_driver::AutoRest;
using braco_driver::ControlLease;
using mycobot::TempLevel;

namespace
{

const ControlLease::Clock::time_point T0{};

}  // namespace

TEST(ControlLease, StartsFreeAndAcceptsNobody)
{
  ControlLease lease;
  EXPECT_EQ(lease.owner(), "");
  EXPECT_FALSE(lease.accepts("web"));
  EXPECT_FALSE(lease.accepts(""));
}

TEST(ControlLease, FreeIsTaken)
{
  ControlLease lease;
  const auto r = lease.take("web", false, T0);
  EXPECT_TRUE(r.success);
  EXPECT_EQ(r.owner, "web");
  EXPECT_EQ(r.previous, "");
  EXPECT_TRUE(lease.accepts("web"));
  EXPECT_FALSE(lease.accepts("joystick"));
}

TEST(ControlLease, OtherWithoutForceIsRefused)
{
  ControlLease lease;
  lease.take("web", false, T0);
  const auto r = lease.take("joystick", false, T0 + 1s);
  EXPECT_FALSE(r.success);
  EXPECT_EQ(r.owner, "web");
  EXPECT_TRUE(lease.accepts("web"));
}

TEST(ControlLease, ForceChangesOwner)
{
  ControlLease lease;
  lease.take("web", false, T0);
  const auto r = lease.take("joystick", true, T0 + 1s);
  EXPECT_TRUE(r.success);
  EXPECT_EQ(r.owner, "joystick");
  EXPECT_EQ(r.previous, "web");
  EXPECT_FALSE(lease.accepts("web"));
}

TEST(ControlLease, OwnerTakingAgainKeepsIt)
{
  ControlLease lease;
  lease.take("web", false, T0);
  const auto r = lease.take("web", false, T0 + 4s);
  EXPECT_TRUE(r.success);
  EXPECT_EQ(r.previous, "web");
  // The second take renewed the lease
  EXPECT_EQ(lease.expireIdle(T0 + 8s, 5s), "");
}

TEST(ControlLease, EmptySourceNeverTakes)
{
  ControlLease lease;
  EXPECT_FALSE(lease.take("", true, T0).success);
  EXPECT_EQ(lease.owner(), "");
}

TEST(ControlLease, IdleOwnerIsReleased)
{
  ControlLease lease;
  lease.take("joystick", false, T0);
  lease.touch(T0 + 3s);
  EXPECT_EQ(lease.expireIdle(T0 + 7s, 5s), "");   // 4 s since the last motion
  EXPECT_EQ(lease.expireIdle(T0 + 9s, 5s), "joystick");
  EXPECT_EQ(lease.owner(), "");
  EXPECT_EQ(lease.expireIdle(T0 + 20s, 5s), "");   // already free
}

TEST(ControlLease, OnlyTheOwnerReleases)
{
  ControlLease lease;
  lease.take("web", false, T0);
  EXPECT_FALSE(lease.release("joystick"));
  EXPECT_FALSE(lease.release(""));
  EXPECT_EQ(lease.owner(), "web");
  EXPECT_TRUE(lease.release("web"));
  EXPECT_EQ(lease.owner(), "");
  EXPECT_FALSE(lease.release("web"));
}

TEST(AutoRest, RestsOncePerHotEpisode)
{
  AutoRest rest;
  EXPECT_FALSE(rest.update(TempLevel::Normal));
  EXPECT_FALSE(rest.update(TempLevel::Warm));
  EXPECT_TRUE(rest.update(TempLevel::Hot));
  EXPECT_TRUE(rest.active());
  // Still hot (for example after make resume): not again
  EXPECT_FALSE(rest.update(TempLevel::Hot));
  EXPECT_FALSE(rest.update(TempLevel::Hot));
}

TEST(AutoRest, EpisodeEndsBelowHot)
{
  AutoRest rest;
  EXPECT_TRUE(rest.update(TempLevel::Hot));
  EXPECT_FALSE(rest.update(TempLevel::Warm));
  EXPECT_FALSE(rest.active());
  EXPECT_TRUE(rest.update(TempLevel::Hot));   // a new episode
}

TEST(AutoRest, RetryAfterAFailedRequest)
{
  AutoRest rest;
  EXPECT_TRUE(rest.update(TempLevel::Hot));
  rest.retry();
  EXPECT_TRUE(rest.update(TempLevel::Hot));
}
