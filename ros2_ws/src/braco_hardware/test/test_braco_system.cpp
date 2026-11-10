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

// BracoSystem (#87) loaded by pluginlib from the real-arm URDF, against arm_driver on PtyArm
// (mycobot_cpp/test/pty_arm.hpp): read() must give the fake arm's angles, in radians.

#include <gtest/gtest.h>

#include "hardware_interface/system_interface.hpp"
#include "pluginlib/class_loader.hpp"

TEST(BracoSystem, PluginIsExported)
{
  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader(
    "hardware_interface", "hardware_interface::SystemInterface");
  EXPECT_TRUE(loader.isClassAvailable("braco_hardware/BracoSystem"));
  EXPECT_NE(loader.createUniqueInstance("braco_hardware/BracoSystem"), nullptr);
}
