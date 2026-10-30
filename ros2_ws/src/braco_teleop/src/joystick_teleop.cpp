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


// joystick_teleop - drives the arm from a joystick through arm_driver. The node is
// braco_teleop::JoystickTeleop (include/braco_teleop/joystick_teleop.hpp).

#include <memory>

#include "braco_teleop/joystick_teleop.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<braco_teleop::JoystickTeleop>());
  rclcpp::shutdown();
  return 0;
}
