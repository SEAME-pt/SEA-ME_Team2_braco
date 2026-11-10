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

#include <array>
#include <chrono>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>

#include "ament_index_cpp/get_package_share_path.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace
{

// The URDF of braco.launch.py sim:=false: braco.urdf.xacro with real_arm:=true, through xacro.
std::string realArmUrdf()
{
  const auto xacro_file =
    ament_index_cpp::get_package_share_path("braco_description") / "urdf" / "braco.urdf.xacro";
  const std::string command = "xacro " + xacro_file.string() + " real_arm:=true";
  std::unique_ptr<FILE, int (*)(FILE *)> pipe(popen(command.c_str(), "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("cannot run: " + command);
  }
  std::string urdf;
  std::array<char, 4096> buffer{};
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
    urdf += buffer.data();
  }
  return urdf;
}

// The <ros2_control> block named BracoSystem in that URDF.
hardware_interface::HardwareInfo bracoSystemInfo()
{
  for (const auto & info : hardware_interface::parse_control_resources_from_urdf(realArmUrdf())) {
    if (info.name == "BracoSystem") {
      return info;
    }
  }
  throw std::runtime_error("no BracoSystem <ros2_control> block in the real-arm URDF");
}

class BracoSystemTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  // The plugin as ros2_control_node loads it: by name, init() with our executor, its state
  // interfaces exported. The executor spins in its own thread, like the controller manager's.
  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    system_ = loader_.createUniqueInstance("braco_hardware/BracoSystem");

    hardware_interface::HardwareComponentParams params;
    params.hardware_info = bracoSystemInfo();
    params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    params.logger = rclcpp::get_logger("test_braco_system");
    params.executor = executor_;
    ASSERT_EQ(system_->init(params), CallbackReturn::SUCCESS);
    system_->on_export_state_interfaces();

    spinner_ = std::thread([this] {executor_->spin();});
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spinner_.joinable()) {spinner_.join();}
    system_.reset();   // before loader_, which owns the plugin library
  }

  using State = lifecycle_msgs::msg::State;

  // The lifecycle callbacks, called directly with the state the plugin comes from
  CallbackReturn configure()
  {
    return system_->on_configure(
      rclcpp_lifecycle::State(State::PRIMARY_STATE_UNCONFIGURED, "unconfigured"));
  }
  CallbackReturn activate()
  {
    return system_->on_activate(
      rclcpp_lifecycle::State(State::PRIMARY_STATE_INACTIVE, "inactive"));
  }
  return_type read() {return system_->read(rclcpp::Time(0, 0, RCL_STEADY_TIME), 10ms);}

  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader_{
    "hardware_interface", "hardware_interface::SystemInterface"};
  pluginlib::UniquePtr<hardware_interface::SystemInterface> system_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread spinner_;
};

}  // namespace

// Nothing publishes /joint_states: read() must refuse, not report stale or zero joints.
TEST_F(BracoSystemTest, ReadFailsBeforeFirstJointStates)
{
  ASSERT_EQ(configure(), CallbackReturn::SUCCESS);
  EXPECT_EQ(read(), return_type::ERROR);
}

// No arm_driver: on_activate waits its 2 s for the first /joint_states, then fails.
TEST_F(BracoSystemTest, ActivateFailsWithoutJointStates)
{
  ASSERT_EQ(configure(), CallbackReturn::SUCCESS);
  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(activate(), CallbackReturn::ERROR);
  EXPECT_GE(std::chrono::steady_clock::now() - start, 1900ms);
}
