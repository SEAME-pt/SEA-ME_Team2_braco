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

// zero - boot routine: send the arm to the zero pose (all joints at 0 degrees).
//
// ROS 2 version of libs/cpp/srcs/zero.cpp: goes through arm_driver instead of opening the
// serial port. Same pose (mycobot::ZERO_ANGLES) and speed. Run by hand with `make zero`.

#include "braco_bringup/arm_moves.hpp"
#include "mycobot.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr int SPEED = 30;
constexpr double TOLERANCE_DEG = 2.0;
constexpr auto TIMEOUT = 20s;
constexpr auto DRIVER_TIMEOUT = 60s;   // at boot arm_driver may still be starting

bool zero(braco_bringup::ArmMoves & arm)
{
  if (!arm.waitForDriver(DRIVER_TIMEOUT)) {return false;}
  // Leave rest mode (set by park), otherwise power_on and the move are refused.
  arm.setRest(false);
  arm.powerOn();   // the joints may still be free from the last park
  if (arm.moveTo(mycobot::ZERO_ANGLES, SPEED, TOLERANCE_DEG, TIMEOUT)) {
    RCLCPP_INFO(arm.get_logger(), "At zero");
    return true;
  }
  RCLCPP_ERROR(arm.get_logger(), "Zero position not reached - arm stopped");
  return false;
}

}  // namespace

int main(int argc, char ** argv)
{
  return braco_bringup::run(argc, argv, "arm_zero", zero);
}
