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

#include "braco_driver/temperature_monitor.hpp"

#include <sys/wait.h>
#include <unistd.h>

#include <memory>
#include <string>
#include <vector>

using namespace std::chrono_literals;
using diagnostic_msgs::msg::DiagnosticStatus;

namespace braco_driver
{

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

TemperatureMonitor::TemperatureMonitor(const rclcpp::NodeOptions & options)
: Node("temperature_monitor", options),
  lcd_(declare_parameter<std::string>("lcd_bus", ui::LcdDisplay::DEFAULT_BUS))
{
  stale_timeout_ = std::chrono::duration<double>(
    declare_parameter<double>("stale_timeout_s", 5.0));
  auto_rest_on_hot_ = declare_parameter<bool>("auto_rest_on_hot", true);
  park_on_hot_ = declare_parameter<bool>("park_on_hot", true);
  park_command_ = declare_parameter<std::vector<std::string>>(
    "park_command", {"ros2", "run", "braco_bringup", "park"});

  if (!lcd_.ok()) {
    RCLCPP_WARN(
      get_logger(), "LCD not available (%s) - publishing /diagnostics only",
      lcd_.error().c_str());
  }
  setColour(status_, mycobot::TempLevel::Normal);
  status_.feedbackOk = false;   // nothing received yet
  lcd_.show(status_, true);

  diag_pub_ = create_publisher<diagnostic_msgs::msg::DiagnosticArray>("diagnostics", 10);
  rest_client_ = create_client<std_srvs::srv::SetBool>("arm/set_read_only");
  temp_sub_ = create_subscription<ServoTemperatures>(
    "arm/servo_temperatures", 10,
    [this](ServoTemperatures::ConstSharedPtr msg) {onTemps(*msg);});
  stale_timer_ = create_wall_timer(1s, [this]() {checkStale();});
  last_msg_ = std::chrono::steady_clock::now();
}

void TemperatureMonitor::onTemps(const ServoTemperatures & msg)
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

  if (auto_rest_on_hot_ && auto_rest_.update(temps[hottest])) {
    requestRest(hottest + 1, temps[hottest]);
  }

  status_.feedbackOk = true;
  status_.temps = temps;
  status_.hotJoint = (level == mycobot::TempLevel::Hot) ? hottest + 1 : 0;
  status_.autoRest = status_.hotJoint != 0 && auto_rest_.active();
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
        get_logger(), "servo J%d is HOT (%d C)",
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

void TemperatureMonitor::requestRest(int joint, int celsius)
{
  if (park_on_hot_ && !park_command_.empty()) {
    hot_joint_ = joint;
    hot_celsius_ = celsius;
    RCLCPP_ERROR(
      get_logger(), "servo J%d is HOT (%d C): parking the arm and releasing the joints "
      "(make resume to leave rest)", joint, celsius);
    if (startPark()) {return;}
    RCLCPP_ERROR(get_logger(), "park could not start: falling back to rest mode only");
  }
  requestRestMode(joint, celsius);
}

bool TemperatureMonitor::startPark()
{
  if (park_pid_ > 0) {return true;}   // one is already running
  std::vector<char *> argv;
  argv.reserve(park_command_.size() + 1);
  // execvp takes char *const[], so `arg` cannot be a const reference (data() would be const)
  // cppcheck-suppress constVariableReference
  for (auto & arg : park_command_) {
    argv.push_back(arg.data());
  }
  argv.push_back(nullptr);
  const pid_t pid = fork();
  if (pid < 0) {return false;}
  if (pid == 0) {
    execvp(argv[0], argv.data());
    _exit(127);   // exec failed
  }
  park_pid_ = pid;
  return true;
}

void TemperatureMonitor::checkPark()
{
  if (park_pid_ <= 0) {return;}
  int status = 0;
  if (waitpid(park_pid_, &status, WNOHANG) != park_pid_) {return;}
  park_pid_ = -1;
  if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
    // park enters rest mode itself; asking again is harmless and makes the order certain:
    // park first, rest mode after, whatever park did at its end
    RCLCPP_INFO(get_logger(), "park finished: joints released, confirming rest mode");
    requestRestMode(hot_joint_, hot_celsius_);
    return;
  }
  RCLCPP_ERROR(get_logger(), "park failed (status %d): putting the arm to rest", status);
  requestRestMode(hot_joint_, hot_celsius_);
}

void TemperatureMonitor::requestRestMode(int joint, int celsius)
{
  if (!rest_client_->service_is_ready()) {
    RCLCPP_ERROR(
      get_logger(), "servo J%d is HOT (%d C) but arm_driver is not reachable: "
      "cannot put the arm to rest", joint, celsius);
    auto_rest_.retry();
    return;
  }
  RCLCPP_ERROR(
    get_logger(), "servo J%d is HOT (%d C): putting the arm to rest (make resume to leave it)",
    joint, celsius);
  auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
  request->data = true;
  rest_client_->async_send_request(
    request, [this](rclcpp::Client<std_srvs::srv::SetBool>::SharedFuture future) {
      const auto result = future.get();
      if (!result->success) {
        RCLCPP_ERROR(get_logger(), "rest mode not set: %s", result->message.c_str());
        auto_rest_.retry();
      }
    });
}

void TemperatureMonitor::checkStale()
{
  checkPark();
  if (std::chrono::steady_clock::now() - last_msg_ > stale_timeout_) {
    showNoFeedback("no temperatures from arm_driver - is it running?");
  }
}

void TemperatureMonitor::showNoFeedback(const char * why)
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

}  // namespace braco_driver
