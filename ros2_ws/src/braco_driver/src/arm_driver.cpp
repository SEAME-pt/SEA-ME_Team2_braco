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

// arm_driver - the only process that talks to the myCobot 280 serial port. The node is
// braco_driver::ArmDriver (include/braco_driver/arm_driver.hpp).

#include <exception>
#include <memory>

#include "braco_driver/arm_driver.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<braco_driver::ArmDriver>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("arm_driver"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
