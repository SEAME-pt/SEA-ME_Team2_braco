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

// arm_driver - the only process that talks to the myCobot 280 serial port.
//
// Every other node (temperature monitor, joystick teleop, later the automatic
// mode) goes through the topics and services below, so they can run, stop and
// crash independently while the port has a single owner.
//
//   pub  /joint_states               sensor_msgs/JointState (radians)
//   pub  /arm/servo_temperatures     braco_interfaces/ServoTemperatures
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
// Control owner (docs/ADR-0002-controle-unico.md): the web page, the joystick,
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
//   4. every temp_period_s read the servo temperatures and publish them
//   5. watchdog: no new motion command for command_timeout_ms after the arm was
//      told to move -> stop once. Together with the step-ahead targets of the
//      teleop node this bounds how far the arm coasts if a client dies.
// The driver never powers the arm on or moves it by itself.

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <utility>

#include "braco_driver/control.hpp"
#include "mycobot.hpp"
#include "rclcpp/rclcpp.hpp"
#include "braco_interfaces/msg/arm_command.hpp"
#include "braco_interfaces/msg/servo_temperatures.hpp"
#include "braco_interfaces/srv/take_control.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/color_rgba.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_srvs/srv/trigger.hpp"

using namespace std::chrono_literals;
using braco_interfaces::msg::ArmCommand;
using braco_interfaces::msg::ServoTemperatures;
using braco_interfaces::srv::TakeControl;

namespace
{

constexpr auto COMMAND_GAP = 50ms;   // pause after each motion command
constexpr int ANGLE_READ_RETRIES = 1;
constexpr double DEG_TO_RAD = M_PI / 180.0;

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

}  // namespace

class ArmDriver : public rclcpp::Node
{
public:
  ArmDriver()
  : Node("arm_driver")
  {
    const auto port = declare_parameter<std::string>(
      "port", mycobot::MyCobot280::DEFAULT_PORT);
    const auto baud = declare_parameter<int>("baud", mycobot::MyCobot280::DEFAULT_BAUD);
    state_period_ = std::chrono::duration<double>(
      1.0 / declare_parameter<double>("state_rate_hz", 20.0));
    temp_period_ = std::chrono::duration<double>(declare_parameter<double>("temp_period_s", 2.0));
    command_timeout_ = std::chrono::milliseconds(
      declare_parameter<int>("command_timeout_ms", 300));
    gripper_speed_ = declare_parameter<int>("gripper_speed", 50);
    control_idle_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(declare_parameter<double>("control_idle_s", 5.0)));

    // Throws if the port cannot be opened: main() reports it and exits non-zero.
    mc_ = std::make_unique<mycobot::MyCobot280>(port, static_cast<int>(baud));
    RCLCPP_INFO(get_logger(), "serial port %s open at %ld baud", port.c_str(), baud);

    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    temp_pub_ = create_publisher<ServoTemperatures>("arm/servo_temperatures", 10);

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

  ~ArmDriver() override
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

private:
  // ---- Callbacks (executor thread): only record what was asked -------------

  // Rest mode: only monitoring. Entering it drops whatever motion was pending.
  void setReadOnly(bool on)
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

  // True (and the response filled in) when rest mode refuses `what`.
  bool refuseInRest(bool & success, std::string & message, const char * what)
  {
    if (!read_only_) {return false;}
    success = false;
    message = std::string(what) + " refused: rest mode (call arm/set_read_only false)";
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
    return true;
  }

  void publishOwner(const std::string & owner)
  {
    std_msgs::msg::String msg;
    msg.data = owner;
    owner_pub_->publish(msg);
  }

  void onTakeControl(const TakeControl::Request & req, TakeControl::Response & res)
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
    }
    if (changed) {
      publishOwner(owner);
      RCLCPP_INFO(
        get_logger(), "control: %s (%s)",
        owner.empty() ? "free" : owner.c_str(), res.message.c_str());
    } else if (!res.success) {
      RCLCPP_WARN(get_logger(), "take_control from '%s': %s", req.source.c_str(),
        res.message.c_str());
    }
  }

  // The serial thread: free the control after control_idle_s without motion.
  void expireControl()
  {
    std::string dropped;
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      dropped = control_.expireIdle(std::chrono::steady_clock::now(), control_idle_);
      if (!dropped.empty()) {last_owner_.clear();}
    }
    if (dropped.empty()) {return;}
    publishOwner("");
    RCLCPP_INFO(
      get_logger(), "control: free (%s idle for %.1f s)", dropped.c_str(),
      std::chrono::duration<double>(control_idle_).count());
  }

  void onCommand(const ArmCommand & msg)
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

  void onLed(const std_msgs::msg::ColorRGBA & msg)
  {
    auto to8 = [](float v) {
        return static_cast<uint8_t>(std::lround(std::clamp(v, 0.0f, 1.0f) * 255.0f));
      };
    std::lock_guard<std::mutex> lock(state_mutex_);
    pending_led_ = std::array<uint8_t, 3>{to8(msg.r), to8(msg.g), to8(msg.b)};
  }

  void clearPending()
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    pending_joints_.clear();
    pending_all_.reset();
    moving_ = false;
  }

  // Run one serial action for a service and fill its response.
  template<typename F>
  void runSerial(bool & success, std::string & message, const char * what, F action)
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

  // ---- Serial thread -------------------------------------------------------

  void serialLoop()
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

  // Stop, LED and at most one motion command. True if a motion command was sent.
  bool serviceCommands()
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

  void publishAngles()
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

  void publishTemps()
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

  void checkWatchdog()
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

  braco_driver::ControlLease control_;   // under state_mutex_
  std::string last_owner_;               // last value published on arm/control_owner

  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Publisher<ServoTemperatures>::SharedPtr temp_pub_;
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

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int code = 0;
  try {
    rclcpp::spin(std::make_shared<ArmDriver>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("arm_driver"), "%s", e.what());
    code = 1;
  }
  rclcpp::shutdown();
  return code;
}
