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

// ArmMoves - small client of arm_driver for zero and park (C++ version of the old arm_moves.py).
//
// The programs cannot open /dev/ttyTHS1 like mycobot_cpp/srcs/zero.cpp and park.cpp do: arm_driver
// owns the port. Instead they:
//   * wait for arm_driver (its /joint_states and services),
//   * take the control as "script", forced (a person ran make zero / park on purpose: the web
//     page or the joystick lose it), and give it back when done, also on failure (run()),
//   * send bounded steps toward the pose (at most STEP_DEG ahead of the real angles, all joints
//     arriving together), resent every RESEND, so the driver's watchdog stops the arm if the
//     program dies half-way,
//   * check the real angles until the pose is reached or the timeout expires.

#ifndef BRACO_BRINGUP__ARM_MOVES_HPP_
#define BRACO_BRINGUP__ARM_MOVES_HPP_

#include <optional>   // cpplint takes it for a C header: it must come first

#include <array>
#include <chrono>
#include <memory>
#include <string>

#include "braco_interfaces/msg/arm_command.hpp"
#include "braco_interfaces/srv/take_control.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace braco_bringup
{

using Angles = std::array<double, 6>;   // J1..J6 in degrees

// How long ArmMoves waits. The programs keep the defaults; the tests shorten them.
struct Timeouts
{
  std::chrono::duration<double> call{5.0};   // one service call to arm_driver
  std::chrono::duration<double> move{20.0};  // zero / park reaching the pose
};

class ArmMoves : public rclcpp::Node
{
public:
  explicit ArmMoves(const std::string & name, Timeouts timeouts = {});

  const Timeouts & timeouts() const {return timeouts_;}

  // Wait until arm_driver publishes angles and offers its services.
  bool waitForDriver(std::chrono::duration<double> timeout);

  // Take the control from whoever has it (forced). False if arm_driver refused or did not answer.
  bool takeControl();
  // Give it back, if this program has it. Best effort: arm_driver frees it when idle anyway.
  void releaseControl();

  bool powerOn();
  bool releaseServos();

  // Rest mode: the driver keeps publishing temperatures but refuses motion.
  // Best effort (an older driver has no such service): a failure only logs.
  bool setRest(bool on);

  // Move all joints to `pose`. True once every joint is within tolerance_deg.
  // On timeout or lost feedback the arm is stopped and false returned.
  bool moveTo(
    const Angles & pose, int speed, double tolerance_deg,
    std::chrono::duration<double> timeout);

  void stop();

  // Sleep while still processing callbacks.
  void wait(std::chrono::duration<double> duration);

private:
  // Real joint angles in degrees, or nullopt if they are not fresh.
  std::optional<Angles> angles() const;
  void onJoints(const sensor_msgs::msg::JointState & msg);
  bool callTrigger(rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr client, const char * what);

  std::optional<Angles> angles_;
  std::chrono::steady_clock::time_point angles_time_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<braco_interfaces::msg::ArmCommand>::SharedPtr command_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_client_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr release_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr rest_client_;
  rclcpp::Client<braco_interfaces::srv::TakeControl>::SharedPtr control_client_;
  Timeouts timeouts_;
  bool has_control_ = false;
  // Runs the callbacks while a move or a call waits (rclcpp::spin_some is deprecated)
  rclcpp::executors::SingleThreadedExecutor executor_;
};

// Init ROS, run action(node) -> bool, shut down; return a process exit code.
template<typename Action>
int run(int argc, char ** argv, const std::string & name, Action action)
{
  rclcpp::init(argc, argv);
  int code = 1;
  {
    auto node = std::make_shared<ArmMoves>(name);
    try {
      code = action(*node) ? 0 : 1;
    } catch (const std::exception & e) {
      // Interrupted or failed: arm_driver's watchdog stops the arm.
      RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    }
    if (rclcpp::ok()) {node->releaseControl();}
  }
  rclcpp::shutdown();
  return code;
}

}  // namespace braco_bringup

#endif  // BRACO_BRINGUP__ARM_MOVES_HPP_
