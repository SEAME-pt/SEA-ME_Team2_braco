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

// ArmDriver - the only process that talks to the myCobot 280 serial port.
//
// Every other node (temperature monitor, joystick teleop, later the automatic
// mode) goes through the topics and services below, so they can run, stop and
// crash independently while the port has a single owner.
//
//   pub  /joint_states               sensor_msgs/JointState (radians)
//   pub  /arm/servo_temperatures     braco_interfaces/ServoTemperatures
//   pub  /arm/servo_voltages         braco_interfaces/ServoVoltages
//   sub  /arm/command                braco_interfaces/ArmCommand (latest wins)
//   sub  /arm/led                    std_msgs/ColorRGBA (0..1 per channel)
//   srv  /arm/power_on               std_srvs/Trigger
//   srv  /arm/release_servos         std_srvs/Trigger
//   srv  /arm/set_gripper            std_srvs/SetBool (true = close)
//   srv  /arm/set_read_only          std_srvs/SetBool (true = rest mode)
//   pub  /arm/read_only              std_msgs/Bool (latched)
//   srv  /arm/take_control           braco_interfaces/TakeControl
//   pub  /arm/control_owner          std_msgs/String (latched, "" = free)
//
// Control owner (docs/ADR-0002-unique-control.md): the web page, the joystick,
// zero/park and the mission never move the arm at the same time. Motion is obeyed
// only from the ArmCommand.source that holds the control (ControlLease); STOP always
// passes, whoever sends it. The owner takes it with /arm/take_control (force takes
// it from another owner and drops that owner's pending motion) and loses it after
// control_idle_s without motion. power_on, set_gripper and release_servos carry no
// source: clients call them only while they own the control.
//
// Rest mode (read_only): motion, gripper and power_on are refused, but the
// serial loop keeps publishing angles and temperatures, so the arm can cool
// down under watch. park enters it; set_read_only false (or make resume) leaves it.
//
// One serial thread does all the work, in this order on every pass:
//   1. a requested stop (drops everything pending)
//   2. one pending motion command, then COMMAND_GAP (as the C++ programs do)
//   3. read the angles and publish /joint_states
//   4. every temp_period_s read the servo temperatures and voltages and publish them
//   5. watchdog: no new motion command for command_timeout_ms after the arm was
//      told to move -> stop once. Together with the step-ahead targets of the
//      teleop node this bounds how far the arm coasts if a client dies.
// The driver never powers the arm on or moves it by itself.

#ifndef BRACO_DRIVER__ARM_DRIVER_HPP_
#define BRACO_DRIVER__ARM_DRIVER_HPP_

#include <optional>   // cpplint takes it for a C header: it must come first

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "braco_driver/control.hpp"
#include "braco_interfaces/msg/arm_command.hpp"
#include "braco_interfaces/msg/servo_temperatures.hpp"
#include "braco_interfaces/msg/servo_voltages.hpp"
#include "braco_interfaces/srv/take_control.hpp"
#include "mycobot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace braco_driver
{

class ArmDriver : public rclcpp::Node
{
public:
  // Throws if the port cannot be opened.
  explicit ArmDriver(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~ArmDriver() override;

private:
  using ArmCommand = braco_interfaces::msg::ArmCommand;
  using ServoTemperatures = braco_interfaces::msg::ServoTemperatures;
  using ServoVoltages = braco_interfaces::msg::ServoVoltages;
  using TakeControl = braco_interfaces::srv::TakeControl;

  struct Target
  {
    double angle;
    int speed;
  };

  struct AllTarget
  {
    mycobot::Angles angles;
    int speed;
  };

  // ---- Callbacks (executor thread): only record what was asked -------------

  // Rest mode: only monitoring. Entering it drops whatever motion was pending.
  void setReadOnly(bool on);
  // True (and the response filled in) when rest mode refuses `what`.
  bool refuseInRest(bool & success, std::string & message, const char * what);
  void publishOwner(const std::string & owner);
  void onTakeControl(const TakeControl::Request & req, TakeControl::Response & res);
  // The serial thread: free the control after control_idle_s without motion.
  void expireControl();
  void onCommand(const ArmCommand & msg);
  void onLed(const std_msgs::msg::ColorRGBA & msg);
  void clearPending();
  // Run one serial action for a service and fill its response.
  template<typename F>
  void runSerial(bool & success, std::string & message, const char * what, F action);

  // ---- Serial thread -------------------------------------------------------

  void serialLoop();
  // Stop, LED and at most one motion command. True if a motion command was sent.
  bool serviceCommands();
  void publishAngles();
  void publishTemps();
  void publishVoltages();
  void checkWatchdog();

  std::unique_ptr<mycobot::MyCobot280> mc_;
  std::mutex serial_mutex_;   // every mc_ call
  std::mutex state_mutex_;    // everything below that callbacks write
  std::thread serial_thread_;
  std::atomic<bool> running_{true};
  std::atomic<bool> read_only_{false};

  bool stop_requested_ = false;
  std::map<int, Target> pending_joints_;
  std::optional<AllTarget> pending_all_;
  std::optional<std::array<uint8_t, 3>> pending_led_;
  int last_joint_ = 0;
  std::chrono::steady_clock::time_point last_motion_command_{};
  std::atomic<bool> moving_{false};

  std::chrono::duration<double> state_period_{};
  std::chrono::duration<double> temp_period_{};
  std::chrono::milliseconds command_timeout_{};
  std::chrono::steady_clock::duration control_idle_{};
  int gripper_speed_ = 50;

  ControlLease control_;      // under state_mutex_
  std::string last_owner_;    // last value published on arm/control_owner

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<ServoTemperatures>::SharedPtr temp_pub_;
  rclcpp::Publisher<ServoVoltages>::SharedPtr volt_pub_;
  rclcpp::Subscription<ArmCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr led_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr gripper_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr read_only_srv_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr read_only_pub_;
  rclcpp::Service<TakeControl>::SharedPtr take_control_srv_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr owner_pub_;
};

}  // namespace braco_driver

#endif  // BRACO_DRIVER__ARM_DRIVER_HPP_
