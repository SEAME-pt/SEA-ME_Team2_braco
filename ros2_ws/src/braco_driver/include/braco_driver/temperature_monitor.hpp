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

// TemperatureMonitor - ROS 2 version of mycobot_cpp/srcs/temps_lcd.cpp.
//
// Never touches the serial port: it listens to /arm/servo_temperatures from
// arm_driver, so it keeps running whether or not the joystick is in use.
//
//   sub  /arm/servo_temperatures   braco_interfaces/ServoTemperatures
//   pub  /diagnostics              diagnostic_msgs/DiagnosticArray (one status per joint)
//   cli  /arm/set_read_only        rest mode on, when a servo gets hot
//   LCD1602 on I2C, same screens and colours as temps_lcd:
//     |1:31  2:33  3:35|      normal (green), warm (yellow)
//     |4:41  5:38  6:30|
//     |REST: SERVO HOT |      a servo at TEMP_HOT_C or above (red): the arm was put to rest
//     |J4 66 C cooling |      (|SERVO HOT! J4| / |66 C - rest arm| with auto_rest_on_hot false)
//   With no fresh data for stale_timeout_s the LCD shows "NO FEEDBACK".
//
// Auto rest (docs/ADR-0003-rest-per-temp.md): the first hot reading of an episode runs
// park (fold the arm, release the joints so the servos cool, rest mode on), once; the episode
// ends when every joint is below AutoRest::REARM_BELOW_C (60 C, hysteresis: a servo hovering at
// 64/65 C is not parked again on every crossing). With park_on_hot false, or if park cannot run or
// fails, it only calls /arm/set_read_only true. Leaving rest is manual (make resume): the
// monitor never does it.

#ifndef BRACO_DRIVER__TEMPERATURE_MONITOR_HPP_
#define BRACO_DRIVER__TEMPERATURE_MONITOR_HPP_

#include <sys/types.h>

#include <chrono>
#include <string>
#include <vector>

#include "braco_driver/auto_rest.hpp"
#include "braco_interfaces/msg/servo_temperatures.hpp"
#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "lcd_display.hpp"
#include "mycobot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/set_bool.hpp"

namespace braco_driver
{

class TemperatureMonitor : public rclcpp::Node
{
public:
  explicit TemperatureMonitor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  using ServoTemperatures = braco_interfaces::msg::ServoTemperatures;

  void onTemps(const ServoTemperatures & msg);
  // The arm is hot: park it (the park program folds it, releases the joints and enters rest
  // mode). If park cannot run, only rest mode is set.
  void requestRest(int joint, int celsius);
  // Run the park program as a child process. False if it could not be started.
  bool startPark();
  // Called every second: has the park program ended? Then rest mode is requested (also when
  // park failed), so the arm always ends in rest mode.
  void checkPark();
  // Rest mode only (arm_driver stops the arm and refuses motion until make resume).
  void requestRestMode(int joint, int celsius);
  void checkStale();
  // The LCD shows "NO FEEDBACK / check arm power"; /diagnostics reports STALE.
  void showNoFeedback(const char * why);

  ui::LcdDisplay lcd_;
  ui::Status status_;
  std::chrono::duration<double> stale_timeout_{};
  std::chrono::steady_clock::time_point last_msg_{};
  mycobot::TempLevel last_level_ = mycobot::TempLevel::Normal;
  bool had_data_ = false;
  bool warned_ = false;
  bool auto_rest_on_hot_ = true;
  bool park_on_hot_ = true;
  std::vector<std::string> park_command_;
  pid_t park_pid_ = -1;
  int hot_joint_ = 0;     // the servo that started the park, for the fallback message
  int hot_celsius_ = 0;
  AutoRest auto_rest_;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr rest_client_;
  rclcpp::Subscription<ServoTemperatures>::SharedPtr temp_sub_;
  rclcpp::TimerBase::SharedPtr stale_timer_;
};

}  // namespace braco_driver

#endif  // BRACO_DRIVER__TEMPERATURE_MONITOR_HPP_
