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
#include <cstdio>
#include <string>
#include <utility>

#include "lifecycle_msgs/msg/state.hpp"

using namespace std::chrono_literals;

namespace braco_driver
{

namespace
{

constexpr auto COMMAND_GAP = 50ms;   // pause after each motion command
constexpr int ANGLE_READ_RETRIES = 1;
constexpr double DEG_TO_RAD = M_PI / 180.0;
constexpr const char * INACTIVE =
  " refused: arm_driver inactive (ros2 lifecycle set /arm_driver activate)";
constexpr const char * LATCHED = " refused: STOP latched (make resume, or call arm/release_stop)";

// True when every angle of a motion command is a finite number (#142). The library refuses NaN
// and inf too, but only on the serial thread, after the command has fed the watchdog and the lease
bool anglesAreFinite(const braco_interfaces::msg::ArmCommand & msg)
{
  using braco_interfaces::msg::ArmCommand;
  if (msg.type == ArmCommand::JOINTS) {
    return std::all_of(
      msg.targets.begin(), msg.targets.end(),
      [](const auto & t) {return std::isfinite(t.angle_deg);});
  }
  if (msg.type == ArmCommand::ALL_ANGLES) {
    return std::all_of(
      msg.angles_deg.begin(), msg.angles_deg.end(),
      [](double a) {return std::isfinite(a);});
  }
  return true;
}

}  // namespace

ArmDriver::ArmDriver(const rclcpp::NodeOptions & options)
: LifecycleNode("arm_driver", options)
{
  // Declared here, read below: settable before configure (#95)
  declare_parameter<std::string>("port", mycobot::MyCobot280::DEFAULT_PORT);
  declare_parameter<int>("baud", mycobot::MyCobot280::DEFAULT_BAUD);
  declare_parameter<double>("state_rate_hz", 20.0);
  declare_parameter<double>("temp_period_s", 2.0);
  declare_parameter<int>("command_timeout_ms", 300);
  declare_parameter<int>("gripper_speed", 50);
  declare_parameter<double>("control_idle_s", 5.0);
  declare_parameter<double>("stop_hold_s", 3.0);
  declare_parameter<double>("feedback_timeout_s", 1.0);
  declare_parameter<bool>("read_only", false);
}

ArmDriver::CallbackReturn ArmDriver::on_configure(const rclcpp_lifecycle::State &)
{
  const auto port = get_parameter("port").as_string();
  const auto baud = get_parameter("baud").as_int();
  state_period_ = std::chrono::duration<double>(
    1.0 / get_parameter("state_rate_hz").as_double());
  temp_period_ = std::chrono::duration<double>(get_parameter("temp_period_s").as_double());
  command_timeout_ = std::chrono::milliseconds(get_parameter("command_timeout_ms").as_int());
  gripper_speed_ = static_cast<int>(get_parameter("gripper_speed").as_int());
  control_idle_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(get_parameter("control_idle_s").as_double()));
  stop_hold_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(std::max(0.0, get_parameter("stop_hold_s").as_double())));
  feedback_timeout_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(
      std::max(0.0, get_parameter("feedback_timeout_s").as_double())));

  try {
    mc_ = std::make_unique<mycobot::MyCobot280>(port, static_cast<int>(baud));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "cannot open %s: %s", port.c_str(), e.what());
    return CallbackReturn::FAILURE;   // stays unconfigured
  }
  RCLCPP_INFO(get_logger(), "serial port %s open at %ld baud", port.c_str(), baud);

  joint_pub_ = plainPublisher<sensor_msgs::msg::JointState>("joint_states", 10);
  temp_pub_ = plainPublisher<ServoTemperatures>("arm/servo_temperatures", 10);
  volt_pub_ = plainPublisher<ServoVoltages>("arm/servo_voltages", 10);

  stopped_pub_ = plainPublisher<std_msgs::msg::Bool>(
    "arm/stopped", rclcpp::QoS(1).transient_local());
  publishStopped(stopped_);   // a latch survives cleanup (ADR-0005)
  release_stop_srv_ = create_service<std_srvs::srv::Trigger>(
    "arm/release_stop",
    [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr res) {
      releaseStop(res->success, res->message);
    });

  read_only_pub_ = plainPublisher<std_msgs::msg::Bool>(
    "arm/read_only", rclcpp::QoS(1).transient_local());
  setReadOnly(get_parameter("read_only").as_bool());

  read_only_srv_ = create_service<std_srvs::srv::SetBool>(
    "arm/set_read_only",
    [this](std_srvs::srv::SetBool::Request::ConstSharedPtr req,
    std_srvs::srv::SetBool::Response::SharedPtr res) {
      // Leaving rest would not move the arm, but zero/park/scripts do it right before moving
      if (!req->data && refuseWhileStopped(res->success, res->message, "rest mode off")) {
        return;
      }
      setReadOnly(req->data);
      res->success = true;
      res->message = req->data ? "rest mode on" : "rest mode off";
    });

  owner_pub_ = plainPublisher<std_msgs::msg::String>(
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
      // Free joints drop under gravity: that is motion too
      if (refuseWhileStopped(res->success, res->message, "release servos")) {return;}
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

  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_feedback_ = std::chrono::steady_clock::now();   // the timeout counts from the port open
    feedback_lost_ = false;
  }
  link_ok_ = true;   // before the serial thread starts: it is the only one to touch them after
  angle_misses_ = 0;
  running_ = true;
  serial_thread_ = std::thread([this]() {serialLoop();});   // monitoring starts here
  return CallbackReturn::SUCCESS;
}

ArmDriver::CallbackReturn ArmDriver::on_activate(const rclcpp_lifecycle::State &)
{
  active_ = true;
  if (stopped_) {
    RCLCPP_WARN(get_logger(), "active, but STOP latched: motion refused until released");
  } else {
    RCLCPP_INFO(get_logger(), "active: motion accepted");
  }
  return CallbackReturn::SUCCESS;
}

ArmDriver::CallbackReturn ArmDriver::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;   // first: the serial thread drops motion from now on (serviceCommands)
  latchStop("lifecycle");
  // If it fails, motion is refused anyway and the serial thread retries latchStop's stop
  stopArm();
  RCLCPP_WARN(get_logger(), "inactive: arm stopped, monitoring only");
  return CallbackReturn::SUCCESS;
}

ArmDriver::CallbackReturn ArmDriver::on_cleanup(const rclcpp_lifecycle::State &)
{
  stopSerialThread();
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    stop_requested_ = false;
    pending_joints_.clear();
    pending_all_.reset();
    pending_led_.reset();
    control_.clear();
    last_owner_.clear();
  }
  command_sub_.reset();
  led_sub_.reset();
  power_srv_.reset();
  release_srv_.reset();
  gripper_srv_.reset();
  read_only_srv_.reset();
  take_control_srv_.reset();
  release_stop_srv_.reset();
  joint_pub_.reset();
  temp_pub_.reset();
  volt_pub_.reset();
  read_only_pub_.reset();
  owner_pub_.reset();
  stopped_pub_.reset();
  mc_.reset();
  RCLCPP_INFO(get_logger(), "unconfigured: serial port closed");
  return CallbackReturn::SUCCESS;
}

ArmDriver::CallbackReturn ArmDriver::on_shutdown(const rclcpp_lifecycle::State & state)
{
  if (state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE) {on_deactivate(state);}
  if (state.id() != lifecycle_msgs::msg::State::PRIMARY_STATE_UNCONFIGURED) {on_cleanup(state);}
  return CallbackReturn::SUCCESS;
}

ArmDriver::CallbackReturn ArmDriver::on_error(const rclcpp_lifecycle::State & state)
{
  RCLCPP_ERROR(get_logger(), "transition failed: stopping the arm and closing the port");
  active_ = false;
  stopSerialThread();
  stopArm();
  on_cleanup(state);
  return CallbackReturn::SUCCESS;   // back to unconfigured
}

ArmDriver::~ArmDriver()
{
  stopSerialThread();
  stopArm();   // whatever ended the node, leave the arm stopped
}

void ArmDriver::stopSerialThread()
{
  running_ = false;
  if (serial_thread_.joinable()) {serial_thread_.join();}
}

void ArmDriver::stopArm()
{
  if (!mc_) {return;}   // port not open
  try {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    mc_->stop();
    moving_ = false;
    RCLCPP_INFO(get_logger(), "arm stopped");
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "could not send stop: %s", e.what());
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

bool ArmDriver::refuseWhileStopped(bool & success, std::string & message, const char * what)
{
  if (active_ && !stopped_) {return false;}
  success = false;
  message = std::string(what) + (!active_ ? INACTIVE : LATCHED);
  RCLCPP_WARN(get_logger(), "%s", message.c_str());
  return true;
}

bool ArmDriver::refuseInRest(bool & success, std::string & message, const char * what)
{
  if (refuseWhileStopped(success, message, what)) {return true;}
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

void ArmDriver::publishStopped(bool on)
{
  std_msgs::msg::Bool msg;
  msg.data = on;
  stopped_pub_->publish(msg);
}

void ArmDriver::latchStop(const std::string & source)
{
  bool was_stopped = false;
  std::string dropped;
  {
    // Under the lock, like releaseStop: the serial thread latches too (feedback loss, ADR-0007),
    // so a release cannot clear a latch that came in between its check and its clear
    std::lock_guard<std::mutex> lock(state_mutex_);
    was_stopped = stopped_.exchange(true);
    if (!was_stopped) {publishStopped(true);}
    stop_requested_ = true;
    pending_joints_.clear();
    pending_all_.reset();
    last_stop_ = std::chrono::steady_clock::now();   // a second STOP restarts the hold
    dropped = control_.clear();
    if (!dropped.empty() || !last_owner_.empty()) {
      last_owner_.clear();
      publishOwner("");   // under the lock, like the other owner changes
    }
  }
  const char * who = source.empty() ? "?" : source.c_str();
  if (!was_stopped) {
    RCLCPP_WARN(
      get_logger(), "STOP from '%s': latched, motion refused (make resume, after %.1f s)", who,
      std::chrono::duration<double>(stop_hold_).count());
  } else {
    RCLCPP_INFO(get_logger(), "STOP from '%s': already latched, hold restarted", who);
  }
  if (!dropped.empty()) {
    RCLCPP_INFO(get_logger(), "control: free (STOP, was %s)", dropped.c_str());
  }
}

// Check and clear under state_mutex_: the serial thread may latch a STOP at any time (ADR-0007).
void ArmDriver::releaseStop(bool & success, std::string & message)
{
  success = true;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!stopped_) {
      message = "not stopped";
      return;
    }
    const auto left = last_stop_ + stop_hold_ - std::chrono::steady_clock::now();
    if (feedback_lost_) {
      // Released now, the arm would take commands the moment the link comes back
      success = false;
      message = "release refused: no angle feedback from the arm (serial link lost?)";
    } else if (left > std::chrono::steady_clock::duration::zero()) {
      success = false;
      char buf[96];
      std::snprintf(
        buf, sizeof(buf), "release refused: %.1f s left of the hold after the last STOP",
        std::chrono::duration<double>(left).count());
      message = buf;
    } else {
      stopped_ = false;
      publishStopped(false);
    }
  }
  if (!success) {
    RCLCPP_WARN(get_logger(), "%s", message.c_str());
    return;
  }
  message = "stop released (the arm stays still: take the control and send a new command)";
  RCLCPP_WARN(get_logger(), "%s", message.c_str());
}

void ArmDriver::onTakeControl(const TakeControl::Request & req, TakeControl::Response & res)
{
  std::string owner;
  bool changed = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (!req.release && (!active_ || stopped_)) {
      // Even force: a STOP holds until a person releases it (ADR-0005)
      res.success = false;
      res.owner = control_.owner();
      res.message = std::string("take control") + (!active_ ? INACTIVE : LATCHED);
    } else if (req.release) {
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
  } else if (!res.success && (!active_ || stopped_)) {
    // A held stick asks again after every answer while refused
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "take_control from '%s': %s", req.source.c_str(),
      res.message.c_str());
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
  if (msg.type == ArmCommand::STOP) {
    latchStop(msg.source);   // whoever sends it, owner or not
    return;
  }
  if (msg.type == ArmCommand::HALT) {
    // A routine stop: obeyed from anyone, in any state, and it latches nothing
    std::lock_guard<std::mutex> lock(state_mutex_);
    stop_requested_ = true;
    pending_joints_.clear();
    pending_all_.reset();
    return;
  }
  if (!active_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "motion from '%s'%s", msg.source.c_str(), INACTIVE);
    return;
  }
  if (stopped_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "STOP latched: motion from '%s' ignored (make resume, or call arm/release_stop)",
      msg.source.c_str());
    return;
  }
  if (read_only_) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "rest mode: motion ignored (make resume or call arm/set_read_only false)");
    return;
  }
  std::lock_guard<std::mutex> lock(state_mutex_);
  if (!control_.accepts(msg.source)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "motion from '%s' ignored: control owned by '%s'",
      msg.source.c_str(), control_.owner().c_str());
    return;
  }
  // Before touch() and last_motion_command_: a client streaming bad angles must not keep the
  // lease, nor the watchdog from stopping the arm. The whole command goes, not half of it
  if (!anglesAreFinite(msg)) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "motion from '%s' ignored: an angle is not a finite number",
      msg.source.c_str());
    return;
  }
  control_.touch(std::chrono::steady_clock::now());
  switch (msg.type) {
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
        checkFeedback();   // after each read, not once per pass: a pass can hold several reads
        next_state = now + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
          state_period_);
      }
      if (now >= next_temp) {
        publishTemps();
        checkFeedback();
        publishVoltages();
        checkFeedback();
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
    // Outside the try: a port that fails every write never reaches publishAngles
    checkFeedback();
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
  // Checked under serial_mutex_: a target taken before a deactivate never follows its stop
  if (!active_) {return false;}
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
  if (angles) {
    angle_misses_ = 0;
    link_ok_ = true;
  } else if (++angle_misses_ >= 2) {
    link_ok_ = false;
  }
  if (!angles) {
    // No message at all: clients see the state go stale and stop.
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "no valid angle feedback");
    return;
  }
  bool recovered = false;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    last_feedback_ = std::chrono::steady_clock::now();
    recovered = std::exchange(feedback_lost_, false);
    // The latch's stop may have gone into a dead link: send it again now that the arm answers
    if (recovered) {stop_requested_ = true;}
  }
  if (recovered) {
    RCLCPP_WARN(
      get_logger(), "angle feedback is back: stop sent again, STOP still latched (make resume)");
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
  if (link_ok_) {   // else invalid at once: the arm did not answer the last read (ADR-0007)
    std::lock_guard<std::mutex> lock(serial_mutex_);
    temps = mc_->getTemps();
    link_ok_ = temps.has_value();
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
  if (link_ok_) {
    std::lock_guard<std::mutex> lock(serial_mutex_);
    volts = mc_->getVoltages();
    link_ok_ = volts.has_value();
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

void ArmDriver::checkFeedback()
{
  if (feedback_timeout_ <= std::chrono::steady_clock::duration::zero()) {return;}
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (feedback_lost_ ||
      std::chrono::steady_clock::now() - last_feedback_ <= feedback_timeout_)
    {
      return;
    }
    feedback_lost_ = true;
  }
  RCLCPP_ERROR(
    get_logger(), "no angle feedback for %.1f s (serial link lost?): STOP latched",
    std::chrono::duration<double>(feedback_timeout_).count());
  latchStop("feedback");
}

}  // namespace braco_driver
