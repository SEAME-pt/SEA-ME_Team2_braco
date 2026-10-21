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

// temperature_monitor - ROS 2 version of libs/cpp/srcs/temps_lcd.cpp.
//
// Never touches the serial port: it listens to /arm/servo_temperatures from
// arm_driver, so it keeps running whether or not the joystick is in use.
//
//   sub  /arm/servo_temperatures   braco_interfaces/ServoTemperatures
//   pub  /diagnostics              diagnostic_msgs/DiagnosticArray (one status per joint)
//   LCD1602 on I2C, same screens and colours as temps_lcd:
//     |1:31  2:33  3:35|      normal (green), warm (yellow)
//     |4:41  5:38  6:30|
//     |SERVO HOT! J4   |      a servo at TEMP_HOT_C or above (red)
//     |66 C - rest arm |
//   With no fresh data for stale_timeout_s the LCD shows "NO FEEDBACK".

#include <chrono>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_array.hpp"
#include "lcd_display.hpp"
#include "mycobot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "braco_interfaces/msg/servo_temperatures.hpp"

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticStatus;
using braco_interfaces::msg::ServoTemperatures;

namespace
{

// Backlight colour for each temperature level (mycobot::tempColour, as in temps_lcd.cpp)
void setColour(ui::Status & status, mycobot::TempLevel level)
{
  const mycobot::Rgb colour = mycobot::tempColour(level);
  status.r = colour.r;
  status.g = colour.g;
  status.b = colour.b;
}

uint8_t diagLevel(mycobot::TempLevel level)
{
  switch (level) {
    case mycobot::TempLevel::Warm: return DiagnosticStatus::WARN;
    case mycobot::TempLevel::Hot: return DiagnosticStatus::ERROR;
    default: return DiagnosticStatus::OK;
  }
}

const char * levelName(mycobot::TempLevel level)
{
  switch (level) {
    case mycobot::TempLevel::Warm: return "warm";
    case mycobot::TempLevel::Hot: return "HOT";
    default: return "normal";
  }
}

}  // namespace

class TemperatureMonitor : public rclcpp::Node
{
public:
  TemperatureMonitor()
  : Node("temperature_monitor"),
    lcd_(declare_parameter<std::string>("lcd_bus", ui::LcdDisplay::DEFAULT_BUS))
  {
    stale_timeout_ = std::chrono::duration<double>(
      declare_parameter<double>("stale_timeout_s", 5.0));

    if (!lcd_.ok()) {
      RCLCPP_WARN(
        get_logger(), "LCD not available (%s) - publishing /diagnostics only",
        lcd_.error().c_str());
    }
    setColour(status_, mycobot::TempLevel::Normal);
    status_.feedbackOk = false;   // nothing received yet
    lcd_.show(status_, true);

    diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
    temp_sub_ = create_subscription<ServoTemperatures>(
      "arm/servo_temperatures", 10,
      [this](ServoTemperatures::ConstSharedPtr msg) {onTemps(*msg);});
    stale_timer_ = create_wall_timer(1s, [this]() {checkStale();});
    last_msg_ = std::chrono::steady_clock::now();
  }

private:
  void onTemps(const ServoTemperatures & msg)
  {
    last_msg_ = std::chrono::steady_clock::now();
    if (!msg.valid) {
      showNoFeedback("the arm did not answer the temperature read");
      return;
    }

    // The same check as MyCobot280::checkTemps(): the hottest joint sets the level
    mycobot::ServoTemps temps{};
    for (size_t j = 0; j < temps.size(); ++j) {
      temps[j] = msg.celsius[j];
    }
    const auto check = mycobot::evaluateTemps(temps);
    const auto level = check.level;
    const int hottest = check.hottestJoint - 1;

    status_.feedbackOk = true;
    status_.temps = temps;
    status_.hotJoint = (level == mycobot::TempLevel::Hot) ? hottest + 1 : 0;
    setColour(status_, level);
    lcd_.show(status_, true);

    diagnostic_msgs::msg::DiagnosticArray diag;
    diag.header.stamp = msg.header.stamp;
    for (size_t j = 0; j < temps.size(); ++j) {
      const auto joint_level = mycobot::MyCobot280::tempLevel(temps[j]);
      DiagnosticStatus s;
      s.name = "mycobot: J" + std::to_string(j + 1) + " servo temperature";
      s.hardware_id = "mycobot280";
      s.level = diagLevel(joint_level);
      s.message = std::to_string(temps[j]) + " C (" + levelName(joint_level) + ")";
      diagnostic_msgs::msg::KeyValue kv;
      kv.key = "celsius";
      kv.value = std::to_string(temps[j]);
      s.values.push_back(kv);
      diag.status.push_back(s);
    }
    diag_pub_->publish(diag);

    if (level != last_level_ || !had_data_) {
      if (level == mycobot::TempLevel::Hot) {
        RCLCPP_ERROR(
          get_logger(), "servo J%d is HOT (%d C) - let the arm rest",
          hottest + 1, temps[hottest]);
      } else if (level == mycobot::TempLevel::Warm) {
        RCLCPP_WARN(get_logger(), "servo J%d is warm (%d C)", hottest + 1, temps[hottest]);
      } else {
        RCLCPP_INFO(get_logger(), "servo temperatures normal (max %d C)", temps[hottest]);
      }
    }
    last_level_ = level;
    had_data_ = true;
  }

  void checkStale()
  {
    if (std::chrono::steady_clock::now() - last_msg_ > stale_timeout_) {
      showNoFeedback("no temperatures from arm_driver - is it running?");
    }
  }

  // The LCD shows "NO FEEDBACK / check arm power"; /diagnostics reports STALE.
  void showNoFeedback(const char * why)
  {
    if (had_data_ || !warned_) {
      RCLCPP_WARN(get_logger(), "%s", why);
      warned_ = true;
    }
    had_data_ = false;
    status_.feedbackOk = false;
    status_.temps.reset();
    status_.hotJoint = 0;
    lcd_.show(status_, true);

    diagnostic_msgs::msg::DiagnosticArray diag;
    diag.header.stamp = now();
    DiagnosticStatus s;
    s.name = "mycobot: servo temperatures";
    s.hardware_id = "mycobot280";
    s.level = DiagnosticStatus::STALE;
    s.message = why;
    diag.status.push_back(s);
    diag_pub_->publish(diag);
  }

  ui::LcdDisplay lcd_;
  ui::Status status_;
  std::chrono::duration<double> stale_timeout_{};
  std::chrono::steady_clock::time_point last_msg_{};
  mycobot::TempLevel last_level_ = mycobot::TempLevel::Normal;
  bool had_data_ = false;
  bool warned_ = false;

  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::Subscription<ServoTemperatures>::SharedPtr temp_sub_;
  rclcpp::TimerBase::SharedPtr stale_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TemperatureMonitor>());
  rclcpp::shutdown();
  return 0;
}
