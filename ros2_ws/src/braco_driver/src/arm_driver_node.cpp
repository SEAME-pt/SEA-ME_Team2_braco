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

#include "braco_driver/arm_driver.hpp"

#include <algorithm>
#include <cmath>
#include <string>
#include <utility>

using namespace std::chrono_literals;

namespace braco_driver
{

namespace
{

constexpr auto COMMAND_GAP = 50ms;   // pause after each motion command
constexpr int ANGLE_READ_RETRIES = 1;
constexpr double DEG_TO_RAD = M_PI / 180.0;

}  // namespace

ArmDriver::ArmDriver(const rclcpp::NodeOptions & options)
: Node("arm_driver", options)
{
  const auto port = declare_parameter<std::string>(
    "port", mycobot::MyCobot280::DEFAULT_PORT);
  const auto baud = declare_parameter<int>("baud", mycobot::MyCobot280::DEFAULT_BAUD);
  state_period_ = std::chrono::duration<double>(
    1.0 / declare_parameter<double>("state_rate_hz", 20.0));
  temp_period_ = std::chrono::duration<double>(declare_parameter<double>("temp_period_s", 2.0));
  command_timeout_ = std::chrono::milliseconds(
    declare_parameter<int>("command_timeout_ms", 300));
  gripper_speed_ = static_cast<int>(declare_parameter<int>("gripper_speed", 50));
  control_idle_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(declare_parameter<double>("control_idle_s", 5.0)));

  // Throws if the port cannot be opened: main() reports it and exits non-zero.
  mc_ = std::make_unique<mycobot::MyCobot280>(port, static_cast<int>(baud));
  RCLCPP_INFO(get_logger(), "serial port %s open at %ld baud", port.c_str(), baud);

  joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
  temp_pub_ = create_publisher<ServoTemperatures>("arm/servo_temperatures", 10);
  volt_pub_ = create_publisher<ServoVoltages>("arm/servo_voltages", 10);

  read_only_pub_ = create_publisher<std_msgs::msg::Bool>(
    "arm/read_only", rclcpp::QoS(1).transient_local());
  setReadOnly(declare_parameter<bool>("read_only", false));

  read_only_srv_ = create_service<std_srvs::srv::SetBool>(
    "arm/set_read_only",
    [this](std_srvs::srv::SetBool::Request::ConstSharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res) {
      setReadOnly(req->data);
      res->success = true;
      res->message = req->data ? "rest mode on" : "rest mode off";
    });

  owner_pub_ = create_publisher<std_msgs::msg::String>(
    "arm/control_owner", rclcpp::QoS(1).transient_local());
  publishOwner("");
  take_control_srv_ = create_service<TakeControl>(
    "arm/take_control",
    [this](TakeControl::Request::ConstSharedPtr req, TakeControl::Response::SharedPtr res) {
      onTakeControl(*req, *res);
    });

  command_sub_ = create_subscription<ArmCommand>(
    "arm/command", 10, [this](ArmCommand::ConstSharedPtr msg) {onCommand(*msg);});
  led_sub_ = create_subscription<std_msgs::msg::ColorRGBA>(
    "arm/led", 10, [this](std_msgs::msg::ColorRGBA::ConstSharedPtr msg) {onLed(*msg);});

  power_srv_ = create_service<std_srvs::srv::Trigger>(
    "arm/power_on",
    [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
      if (refuseInRest(res->success, res->message, "power on")) {return;}
      runSerial(res->success, res->message, "power on", [this]() {mc_->powerOn();});
    });
  release_srv_ = create_service<std_srvs::srv::Trigger>(
    "arm/release_servos",
    [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
      clearPending();
      runSerial(
        res->success, res->message, "release servos",
        [this]() {mc_->releaseAllServos();});
    });
  gripper_srv_ = create_service<std_srvs::srv::SetBool>(
    "arm/set_gripper",
    [this](std_srvs::srv::SetBool::Request::ConstSharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res) {
      if (refuseInRest(res->success, res->message, "set gripper")) {return;}
      const int state = req->data ? 1 : 0;
      runSerial(
        res->success, res->message, req->data ? "close gripper" : "open gripper",
        [this, state]() {mc_->setGripperState(state, gripper_speed_);});
    });

  serial_thread_ = std::thread([this]() {serialLoop();});
}

ArmDriver::~ArmDriver()
{
  running_ = false;
  if (serial_thread_.joinable()) {serial_thread_.join();}
  // Whatever ended the node, leave the arm stopped.
  try {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    mc_->stop();
    RCLCPP_INFO(get_logger(), "arm stopped");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "could not send final stop: %s", e.what());
  }
}

// ---- Callbacks (executor thread): only record what was asked ---------------

void ArmDriver::setReadOnly(bool on)
{
  const bool changed = read_only_.exchange(on) != on;
  if (on) {clearPending();}
  std_msgs::msg::Bool msg;
  msg.data = on;
  read_only_pub_->publish(msg);
  if (changed || on) {
    RCLCPP_INFO(
      get_logger(), on ? "rest mode ON: monitoring only, motion refused" :
      "rest mode OFF: motion accepted");
  }
}

bool ArmDriver::refuseInRest(bool & success, std::string & message, const char * what)
{
  if (!read_only_) {return false;}
  success = false;
  message = std::string(what) + " refused: rest mode (call arm/set_read_only false)";
  RCLCPP_WARN(get_logger(), "%s", message.c_str());
  return true;
}

void ArmDriver::publishOwner(const std::string & owner)
{
  std_msgs::msg::String msg;
  msg.data = owner;
  owner_pub_->publish(msg);
}

void ArmDriver::onTakeControl(const TakeControl::Request & req, TakeControl::Response & res)
{
  std::string owner;
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (req.release) {
      res.success = control_.release(req.source) || control_.owner().empty();
      res.owner = control_.owner();
      res.message = res.success ? "control released" :
        "only the owner (" + control_.owner() + ") can release the control";
    } else {
      const auto r = control_.take(req.source, req.force, std::chrono::steady_clock::now());
      res.success = r.success;
      res.owner = r.owner;
      if (!r.success) {
        res.message = req.source.empty() ? "source must not be empty" :
          "the arm is controlled by " + r.owner;
      } else if (!r.previous.empty() && r.previous != r.owner) {
        // Forced: whatever the previous owner left pending must not run
        pending_joints_.clear();
        pending_all_.reset();
        stop_requested_ = true;
        res.message = "control taken from " + r.previous;
      } else {
        res.message = "control taken";
      }
    }
    owner = control_.owner();
    changed = owner != last_owner_;
    last_owner_ = owner;
    // Under the lock, so the serial thread's idle release cannot publish out of order
    if (changed) {publishOwner(owner);}
  }
  if (changed) {
    RCLCPP_INFO(
      get_logger(), "control: %s (%s)",
      owner.empty() ? "free" : owner.c_str(), res.message.c_str());
  } else if (!res.success) {
    RCLCPP_WARN(get_logger(), "take_control from '%s': %s", req.source.c_str(),
      res.message.c_str());
  }
}

void ArmDriver::expireControl()
{
  std::string dropped;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    dropped = control_.expireIdle(std::chrono::steady_clock::now(), control_idle_);
    if (!dropped.empty()) {
      last_owner_.clear();
      publishOwner("");
    }
  }
  if (dropped.empty()) {return;}
  RCLCPP_INFO(
    get_logger(), "control: free (%s idle for %.1f s)", dropped.c_str(),
    std::chrono::duration<double>(control_idle_).count());
}

void ArmDriver::onCommand(const ArmCommand & msg)
{
  if (read_only_ && msg.type != ArmCommand::STOP) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "rest mode: motion ignored (make resume or call arm/set_read_only false)");
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (msg.type != ArmCommand::STOP) {
    if (!control_.accepts(msg.source)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "motion from '%s' ignored: control owned by '%s'",
        msg.source.c_str(), control_.owner().c_str());
      return;
    }
    control_.touch(std::chrono::steady_clock::now());
  }
  switch (msg.type) {
    case ArmCommand::STOP:
      stop_requested_ = true;
      pending_joints_.clear();
      pending_all_.reset();
      return;
    case ArmCommand::JOINTS:
      for (const auto & t : msg.targets) {
        if (t.joint < 1 || t.joint > 6) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 2000, "ignoring target for joint %u", t.joint);
          continue;
        }
        pending_joints_[t.joint] = Target{t.angle_deg, t.speed};
      }
      pending_all_.reset();
      break;
    case ArmCommand::ALL_ANGLES: {
        mycobot::Angles angles{};
        std::copy(msg.angles_deg.begin(), msg.angles_deg.end(), angles.begin());
        pending_all_ = AllTarget{angles, msg.speed};
        pending_joints_.clear();
        break;
      }
    default:
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "unknown ArmCommand type %u", msg.type);
      return;
  }
  last_motion_command_ = std::chrono::steady_clock::now();
}

void ArmDriver::onLed(const std_msgs::msg::ColorRGBA & msg)
{
  auto to8 = [](float v) {
      return static_cast<uint8_t>(std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f));
    };
  std::lock_guard<std::mutex> lock(state_mutex_);
  pending_led_ = std::array<uint8_t, 3>{to8(msg.r), to8(msg.g), to8(msg.b)};
}

void ArmDriver::clearPending()
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  pending_joints_.clear();
  pending_all_.reset();
  moving_ = false;
}

template<typename F>
void ArmDriver::runSerial(bool & success, std::string & message, const char * what, F action)
{
  try {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    action();
    success = true;
    message = what;
    RCLCPP_INFO(get_logger(), "%s", what);
  } catch (const std::exception & e) {
    success = false;
    message = std::string(what) + " failed: " + e.what();
    RCLCPP_ERROR(get_logger(), "%s", message.c_str());
  }
}

// ---- Serial thread ---------------------------------------------------------

void ArmDriver::serialLoop()
{
  auto next_state = std::chrono::steady_clock::now();
  auto next_temp = next_state;

  while (running_ && rclcpp::ok()) {
    try {
      const bool sent = serviceCommands();
      const auto now = std::chrono::steady_clock::now();

      if (now >= next_state) {
        publishAngles();
        next_state = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          state_period_);
      }
      if (now >= next_temp) {
        publishTemps();
        publishVoltages();
        next_temp = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          temp_period_);
      }
      checkWatchdog();
      expireControl();

      if (!sent) {std::this_thread::sleep_for(10ms);}
    } catch (const std::exception & e) {
      // A serial error here is not fatal: log it and keep serving.
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "serial error: %s", e.what());
      std::this_thread::sleep_for(100ms);
    }
  }
}

bool ArmDriver::serviceCommands()
{
  bool stop = false;
  std::optional<std::array<uint8_t, 3>> led;
  std::optional<AllTarget> all;
  std::optional<std::pair<int, Target>> joint;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::swap(stop, stop_requested_);
    std::swap(led, pending_led_);
    if (pending_all_) {
      std::swap(all, pending_all_);
    } else if (!pending_joints_.empty()) {
      // Round robin, so a client that resends every tick cannot starve a joint
      auto it = pending_joints_.upper_bound(last_joint_);
      if (it == pending_joints_.end()) {it = pending_joints_.begin();}
      joint = *it;
      last_joint_ = it->first;
      pending_joints_.erase(it);
    }
  }

  std::lock_guard<std::mutex> lock(serial_mutex_);
  if (stop) {
    mc_->stop();
    moving_ = false;
    std::this_thread::sleep_for(COMMAND_GAP);
  }
  if (led) {
    mc_->setColor((*led)[0], (*led)[1], (*led)[2]);
    std::this_thread::sleep_for(COMMAND_GAP);
  }
  if (!all && !joint) {return false;}
  if (all) {
    mc_->sendAngles(all->angles, all->speed);
  } else {
    mc_->sendAngle(joint->first, joint->second.angle, joint->second.speed);
  }
  moving_ = true;
  std::this_thread::sleep_for(COMMAND_GAP);
  return true;
}

void ArmDriver::publishAngles()
{
  std::optional<mycobot::Angles> angles;
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    angles = mc_->getAngles(ANGLE_READ_RETRIES);
  }
  if (!angles) {
    // No message at all: clients see the state go stale and stop.
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no valid angle feedback");
    return;
  }
  sensor_msgs::msg::JointState msg;
  msg.header.stamp = now();
  for (size_t i = 0; i < angles->size(); ++i) {
    msg.name.push_back("joint" + std::to_string(i + 1));
    msg.position.push_back((*angles)[i] * DEG_TO_RAD);
  }
  joint_pub_->publish(msg);
}

void ArmDriver::publishTemps()
{
  std::optional<mycobot::ServoTemps> temps;
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    temps = mc_->getTemps();
  }
  ServoTemperatures msg;
  msg.header.stamp = now();
  msg.valid = temps.has_value();
  if (temps) {
    std::copy(temps->begin(), temps->end(), msg.celsius.begin());
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "no answer to temperature read");
  }
  temp_pub_->publish(msg);
}

void ArmDriver::publishVoltages()
{
  std::optional<mycobot::ServoVolts> volts;
  {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    volts = mc_->getVoltages();
  }
  ServoVoltages msg;
  msg.header.stamp = now();
  msg.valid = volts.has_value();
  if (volts) {
    std::copy(volts->begin(), volts->end(), msg.volts.begin());
  } else {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 10000, "no answer to voltage read");
  }
  volt_pub_->publish(msg);
}

void ArmDriver::checkWatchdog()
{
  bool expired = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    expired = moving_ && pending_joints_.empty() && !pending_all_ &&
      std::chrono::steady_clock::now() - last_motion_command_ > command_timeout_;
  }
  if (expired) {
    RCLCPP_WARN(
      get_logger(), "no motion command for %d ms: stopping the arm",
      static_cast<int>(command_timeout_.count()));
    std::lock_guard<std::mutex> lock(serial_mutex_);
    mc_->stop();
    moving_ = false;
    std::this_thread::sleep_for(COMMAND_GAP);
  }
}

}  // namespace braco_driver
