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

// JoystickTeleop - ROS 2 version of mycobot_cpp/srcs/joystick_all_joints.cpp.
//
// Same mapping, speed modes and safety model (step-ahead jogging: every target
// is only a few degrees ahead of the real angles), but the arm is reached
// through arm_driver instead of the serial port:
//
//   sub  /joint_states        the real angles (older than state_timeout_ms = no feedback)
//   sub  /arm/control_owner   who may move the arm (latched)
//   pub  /arm/command         STOP / JOINTS / ALL_ANGLES, one per tick, source "joystick"
//   pub  /arm/led             speed mode colour (dimmed while another client has the control)
//   cli  /arm/take_control    on the first stick move when the control is free, or forced
//   cli  /arm/power_on        when a joystick connects, once the joystick has the control
//   cli  /arm/set_gripper     A = close, B = open
//
// Control (docs/decisions/ADR-0002-unique-control.md): only the owner moves the arm. With
// the control free, touching a stick, X or A/B takes it. While another client (the
// web page, zero/park) has it, the joystick is ignored until force_button is held for
// force_hold_s. Letting go of everything does not release it: arm_driver frees it
// after control_idle_s without motion.
//
// The joystick may be missing or unplugged at any time: the node then stops the
// arm (if it has the control) and tries to open the device again every second.
//
// Mapping:
//   Left stick  X / Y  joint 1 / joint 2 (base yaw [inverted] / shoulder pitch)
//   Right stick Y / X  joint 4 / joint 5
//   L1 / R1 (6 / 7)    joint 6 (tool rotation), exclusive while held
//   D-pad Y            joint 3 (elbow pitch)
//   A / B              close / open gripper
//   X (click)          home all joints to 0 (click again to abort)
//   Button 9           cycle speed: blue -> green -> purple -> white -> red
//   Button 8 (hold)    take the control from another client (force_button)

#ifndef BRACO_TELEOP__JOYSTICK_TELEOP_HPP_
#define BRACO_TELEOP__JOYSTICK_TELEOP_HPP_

#include <optional>   // cpplint takes it for a C header: it must come first

#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <set>
#include <string>

#include "braco_interfaces/msg/arm_command.hpp"
#include "braco_interfaces/srv/take_control.hpp"
#include "joystick.hpp"
#include "mycobot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "teleop_logic.hpp"

namespace braco_teleop
{

class JoystickTeleop : public rclcpp::Node
{
public:
  explicit JoystickTeleop(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~JoystickTeleop() override;

private:
  using ArmCommand = braco_interfaces::msg::ArmCommand;
  using TakeControl = braco_interfaces::srv::TakeControl;

  // ---- Inputs ----------------------------------------------------------------

  void onJointState(const sensor_msgs::msg::JointState & msg);
  // Angles only if they are fresh enough to jog from (else: no feedback).
  std::optional<mycobot::Angles> freshAngles() const;

  // ---- Control owner ---------------------------------------------------------

  bool isOwner() const;
  void onOwner(const std::string & owner);
  // Ask arm_driver for the control (asynchronous; the answer updates owner_).
  void requestControl(bool force);

  // ---- Outputs ---------------------------------------------------------------

  // Only the owner stops the arm: a STOP from an idle joystick would cut the
  // move of whoever has the control (unplugging it, Ctrl+C...).
  void sendStop();
  void stopArm();
  void setLed(const mycobot::teleop::SpeedMode & mode);
  void setGripper(bool close);

  // ---- Joystick connection ---------------------------------------------------

  // True when a joystick is open. Opens it (at most once per REOPEN_PERIOD).
  bool ensureJoystick();
  void resetState();
  // Lock the joints once the joystick has the control (like powerOn() at start).
  void powerOnIfPending();
  const mycobot::teleop::SpeedMode & mode() const;

  // ---- Control loop (one tick of joystick_all_joints' controlLoop) ----------

  void tick();
  void handleEvent(const input::Event & ev, bool & home_clicked, std::optional<bool> & gripper);
  void home();
  void jog(const mycobot::teleop::Wanted & wanted, const mycobot::Angles & angles);

  // Parameters
  std::string device_;
  std::chrono::milliseconds state_timeout_{};
  bool debug_ = false;
  int force_button_ = mycobot::teleop::BUTTON_FORCE_CONTROL;
  double force_hold_s_ = mycobot::teleop::FORCE_HOLD_S;

  // Control owner (arm/control_owner, or the answer to our take_control)
  std::string owner_;
  bool take_pending_ = false;
  mycobot::teleop::ButtonHold force_hold_;

  // Joystick connection
  std::unique_ptr<input::Joystick> js_;
  std::chrono::steady_clock::time_point next_open_{};
  bool open_warned_ = false;
  bool power_pending_ = false;

  // Latest feedback from arm_driver
  std::optional<mycobot::Angles> angles_;
  std::chrono::steady_clock::time_point angles_time_{};

  // Control state (same as controlLoop's locals)
  size_t mode_index_ = mycobot::teleop::START_MODE;
  std::map<int, int> axis_state_;
  std::map<int, int> button_state_;
  std::set<int> active_;                    // joints commanded on the previous tick
  std::map<int, double> hold_;              // idle joint -> latched angle
  std::map<int, double> last_hold_send_;    // joint -> time of last hold command
  std::set<int> limit_reported_;            // joints whose "at limit" was printed
  bool homing_ = false;
  int bad_reads_ = 0;
  std::map<int, int64_t> axis_changed_ms_;  // axis -> joystick time of last event
  std::set<int> blocked_axes_;              // axes that fired with button 9
  std::optional<int64_t> speed_btn_ms_;     // joystick time of last button 9 event

  rclcpp::Publisher<ArmCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr led_pub_;
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr power_client_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr gripper_client_;
  rclcpp::Client<TakeControl>::SharedPtr take_client_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr owner_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::PreShutdownCallbackHandle shutdown_cb_;
};

}  // namespace braco_teleop

#endif  // BRACO_TELEOP__JOYSTICK_TELEOP_HPP_
