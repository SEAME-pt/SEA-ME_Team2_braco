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
// ROS 2 version of mycobot_cpp/srcs/zero.cpp: goes through arm_driver instead of opening the
// serial port. Same pose (mycobot::ZERO_ANGLES) and speed. Run by hand with `make zero`.

#include "braco_bringup/routines.hpp"

int main(int argc, char ** argv)
{
  return braco_bringup::run(argc, argv, "arm_zero", braco_bringup::zero);
}
