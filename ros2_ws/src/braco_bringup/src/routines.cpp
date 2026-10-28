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

#include "braco_bringup/routines.hpp"

#include <chrono>

#include "mycobot.hpp"

using namespace std::chrono_literals;

namespace braco_bringup
{

namespace
{

constexpr auto ZERO_DRIVER_TIMEOUT = 60s;   // at boot arm_driver may still be starting
constexpr auto PARK_DRIVER_TIMEOUT = 10s;

}  // namespace

bool zero(ArmMoves & arm)
{
  if (!arm.waitForDriver(ZERO_DRIVER_TIMEOUT) || !arm.takeControl()) {return false;}
  // Leave rest mode (set by park), otherwise power_on and the move are refused.
  arm.setRest(false);
  arm.powerOn();   // the joints may still be free from the last park
  if (arm.moveTo(
      mycobot::ZERO_ANGLES, mycobot::MOVE_SPEED, mycobot::POSE_TOLERANCE_DEG, arm.timeouts().move))
  {
    RCLCPP_INFO(arm.get_logger(), "At zero");
    return true;
  }
  RCLCPP_ERROR(arm.get_logger(), "Zero position not reached - arm stopped");
  return false;
}

bool park(ArmMoves & arm)
{
  if (!arm.waitForDriver(PARK_DRIVER_TIMEOUT) || !arm.takeControl()) {return false;}
  arm.setRest(false);   // rest mode refuses the park move itself
  if (!arm.moveTo(
      mycobot::PARK_ANGLES, mycobot::MOVE_SPEED, mycobot::POSE_TOLERANCE_DEG, arm.timeouts().move))
  {
    RCLCPP_ERROR(arm.get_logger(), "Park position not reached - joints left locked");
    return false;
  }
  arm.wait(500ms);   // let the arm settle
  if (!arm.releaseServos()) {return false;}
  // The driver stays up, only watching temperatures, until `make resume`
  arm.setRest(true);
  RCLCPP_INFO(arm.get_logger(), "Parked - joints released, rest mode on");
  return true;
}

}  // namespace braco_bringup
