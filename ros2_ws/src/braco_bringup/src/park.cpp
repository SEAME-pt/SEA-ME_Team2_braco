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

// park - shutdown routine: fold the arm into the park pose, then release all joints.
//
// ROS 2 version of libs/cpp/srcs/park.cpp: goes through arm_driver instead of opening the
// serial port. Same pose (mycobot::PARK_ANGLES) and speed. Run by hand with `make park`
// or before `make down PARK=1`.

#include "braco_bringup/arm_moves.hpp"
#include "mycobot.hpp"

using namespace std::chrono_literals;

namespace
{

constexpr int SPEED = 30;
constexpr double TOLERANCE_DEG = 2.0;
constexpr auto TIMEOUT = 20s;
constexpr auto DRIVER_TIMEOUT = 10s;

bool park(braco_bringup::ArmMoves & arm)
{
  if (!arm.waitForDriver(DRIVER_TIMEOUT)) {return false;}
  arm.setRest(false);   // rest mode refuses the park move itself
  if (!arm.moveTo(mycobot::PARK_ANGLES, SPEED, TOLERANCE_DEG, TIMEOUT)) {
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

}  // namespace

int main(int argc, char ** argv)
{
  return braco_bringup::run(argc, argv, "arm_park", park);
}
