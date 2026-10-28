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


#include "braco_teleop/joystick_teleop.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iterator>

using namespace std::chrono_literals;
using braco_interfaces::msg::JointTarget;
namespace tl = mycobot::teleop;

namespace braco_teleop
{

namespace
{

constexpr auto LOOP_PERIOD = 20ms;
constexpr auto REOPEN_PERIOD = 1s;
constexpr double RAD_TO_DEG = 180.0 / M_PI;
constexpr const char * SOURCE = "joystick";   // ArmCommand.source and the control owner name
constexpr float NOT_OWNER_LED = 0.15f;        // LED brightness while another client has the control

std::string fmtJoints(const std::set<int> & joints)
{
  std::string out = "[";
  for (int j : joints) {
    out += (out.size() > 1 ? ", " : "") + std::to_string(j);
  }
  return out + "]";
}

double nowSeconds()
{
  return std::chrono::duration<double>(
    std::chrono::steady_clock::now().time_since_epoch()).count();
}

}  // namespace

JoystickTeleop::JoystickTeleop(const rclcpp::NodeOptions & options)
: Node("joystick_teleop", options)
{
  device_ = declare_parameter<std::string>("device", input::Joystick::DEFAULT_DEVICE);
  state_timeout_ = std::chrono::milliseconds(declare_parameter<int>("state_timeout_ms", 150));
  debug_ = declare_parameter<bool>("debug", false);
  force_button_ = static_cast<int>(
    declare_parameter<int>("force_button", tl::BUTTON_FORCE_CONTROL));
  force_hold_s_ = declare_parameter<double>("force_hold_s", tl::FORCE_HOLD_S);
  force_hold_ = tl::ButtonHold(force_hold_s_);

  command_pub_ = create_publisher<ArmCommand>("arm/command", 10);
  led_pub_ = create_publisher<std_msgs::msg::ColorRGBA>("arm/led", 10);
  power_client_ = create_client<std_srvs::srv::Trigger>("arm/power_on");
  gripper_client_ = create_client<std_srvs::srv::SetBool>("arm/set_gripper");
  take_client_ = create_client<TakeControl>("arm/take_control");
  owner_sub_ = create_subscription<std_msgs::msg::String>(
    "arm/control_owner", rclcpp::QoS(1).transient_local(),
    [this](std_msgs::msg::String::ConstSharedPtr msg) {onOwner(msg->data);});
  joint_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10,
    [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {onJointState(*msg);});

  timer_ = create_wall_timer(LOOP_PERIOD, [this]() {tick();});
  RCLCPP_INFO(get_logger(), "waiting for joystick %s", device_.c_str());

  // Ctrl+C / docker stop: tell the driver to stop while publishing still works
  // (arm_driver's watchdog stops the arm anyway if this never arrives).
  shutdown_cb_ = get_node_base_interface()->get_context()->add_pre_shutdown_callback(
    [this]() {
      if (!active_.empty() || homing_) {sendStop();}
    });
}

JoystickTeleop::~JoystickTeleop()
{
  get_node_base_interface()->get_context()->remove_pre_shutdown_callback(shutdown_cb_);
}

void JoystickTeleop::onJointState(const sensor_msgs::msg::JointState & msg)
{
  if (msg.position.size() < 6) {return;}
  mycobot::Angles angles{};
  for (size_t i = 0; i < angles.size(); ++i) {
    // Use the names when present, the order otherwise
    size_t idx = i;
    const std::string name = "joint" + std::to_string(i + 1);
    auto it = std::find(msg.name.begin(), msg.name.end(), name);
    if (it != msg.name.end()) {idx = static_cast<size_t>(std::distance(msg.name.begin(), it));}
    if (idx >= msg.position.size()) {return;}
    angles[i] = msg.position[idx] * RAD_TO_DEG;
  }
  angles_ = angles;
  angles_time_ = std::chrono::steady_clock::now();
}

std::optional<mycobot::Angles> JoystickTeleop::freshAngles() const
{
  if (!angles_ || std::chrono::steady_clock::now() - angles_time_ > state_timeout_) {
    return std::nullopt;
  }
  return angles_;
}

// ---- Control owner ---------------------------------------------------------

bool JoystickTeleop::isOwner() const {return owner_ == SOURCE;}

void JoystickTeleop::onOwner(const std::string & owner)
{
  if (owner == owner_) {return;}
  const bool was_owner = isOwner();
  owner_ = owner;
  if (was_owner && !isOwner()) {
    // The new owner (or nobody) moves the arm now: forget the jog without sending STOP
    active_.clear();
    hold_.clear();
    homing_ = false;
    if (owner_.empty()) {
      RCLCPP_INFO(get_logger(), ">>> control released (idle)");
    } else {
      RCLCPP_WARN(
        get_logger(), ">>> control taken by %s: joystick ignored (hold button %d for %.1f s)",
        owner_.c_str(), force_button_, force_hold_s_);
    }
  } else if (isOwner()) {
    RCLCPP_INFO(get_logger(), ">>> joystick has the control");
  }
  if (js_) {setLed(mode());}
}

void JoystickTeleop::requestControl(bool force)
{
  if (take_pending_) {return;}
  if (!take_client_->service_is_ready()) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 2000, "arm_driver not available: cannot take the control");
    return;
  }
  take_pending_ = true;
  auto req = std::make_shared<TakeControl::Request>();
  req->source = SOURCE;
  req->force = force;
  take_client_->async_send_request(
    req, [this](rclcpp::Client<TakeControl>::SharedFuture future) {
      take_pending_ = false;
      const auto res = future.get();
      if (!res->success) {
        RCLCPP_WARN(
          get_logger(), ">>> %s: hold button %d for %.1f s to take it",
          res->message.c_str(), force_button_, force_hold_s_);
      }
      onOwner(res->owner);
    });
}

// ---- Outputs ---------------------------------------------------------------

void JoystickTeleop::sendStop()
{
  if (!isOwner()) {return;}
  ArmCommand cmd;
  cmd.type = ArmCommand::STOP;
  cmd.source = SOURCE;
  command_pub_->publish(cmd);
}

void JoystickTeleop::stopArm()
{
  sendStop();
  active_.clear();
  hold_.clear();
}

void JoystickTeleop::setLed(const tl::SpeedMode & mode)
{
  const float k = (owner_.empty() || isOwner()) ? 1.0f : NOT_OWNER_LED;
  std_msgs::msg::ColorRGBA c;
  c.r = k * static_cast<float>(mode.r) / 255.0f;
  c.g = k * static_cast<float>(mode.g) / 255.0f;
  c.b = k * static_cast<float>(mode.b) / 255.0f;
  c.a = 1.0f;
  led_pub_->publish(c);
}

void JoystickTeleop::setGripper(bool close)
{
  RCLCPP_INFO(get_logger(), ">>> %s gripper", close ? "close" : "open");
  if (!gripper_client_->service_is_ready()) {
    RCLCPP_WARN(get_logger(), "arm_driver not available: gripper command dropped");
    return;
  }
  auto req = std::make_shared<std_srvs::srv::SetBool::Request>();
  req->data = close;
  gripper_client_->async_send_request(req);
}

// ---- Joystick connection ---------------------------------------------------

bool JoystickTeleop::ensureJoystick()
{
  if (js_) {return true;}
  const auto now = std::chrono::steady_clock::now();
  if (now < next_open_) {return false;}
  next_open_ = now + REOPEN_PERIOD;
  try {
    js_ = std::make_unique<input::Joystick>(device_);
  } catch (const std::exception & e) {
    if (!open_warned_) {
      RCLCPP_WARN(get_logger(), "%s - retrying every second", e.what());
      open_warned_ = true;
    }
    return false;
  }
  open_warned_ = false;
  resetState();
  power_pending_ = true;
  RCLCPP_INFO(
    get_logger(), "joystick %s connected. Speed mode: %s (cap %d %%, max coast %.0f deg). "
    "Button 9 changes speed, X homes.",
    device_.c_str(), mode().name, mode().maxSpeed, mode().leadMax);
  setLed(mode());
  return true;
}

void JoystickTeleop::resetState()
{
  axis_state_.clear();
  button_state_.clear();
  active_.clear();
  hold_.clear();
  last_hold_send_.clear();
  limit_reported_.clear();
  axis_changed_ms_.clear();
  blocked_axes_.clear();
  speed_btn_ms_.reset();
  homing_ = false;
  bad_reads_ = 0;
}

void JoystickTeleop::powerOnIfPending()
{
  if (!power_pending_ || !isOwner() || !power_client_->service_is_ready()) {return;}
  power_pending_ = false;
  power_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
}

const tl::SpeedMode & JoystickTeleop::mode() const {return tl::SPEED_MODES[mode_index_];}

// ---- Control loop (one tick of joystick_all_joints' controlLoop) ----------

void JoystickTeleop::tick()
{
  if (!ensureJoystick()) {return;}

  // 1. Drain joystick events
  bool home_clicked = false;
  std::optional<bool> gripper;   // close / open requested
  try {
    for (const input::Event & ev : js_->readEvents()) {
      handleEvent(ev, home_clicked, gripper);
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "%s - stopping the arm, waiting for the joystick", e.what());
    stopArm();
    js_.reset();
    return;
  }

  const tl::Wanted wanted = tl::wantedMotion(axis_state_, button_state_, mode(), blocked_axes_);

  // 2. Control: force_button held takes it; with the control free any input takes it
  if (force_hold_.update(tl::valueOr0(button_state_, force_button_) == 1, nowSeconds()) &&
    !isOwner())
  {
    RCLCPP_INFO(get_logger(), ">>> button %d held: taking the control", force_button_);
    requestControl(true);
  }
  if (!isOwner()) {
    if (!wanted.empty() || home_clicked || gripper) {
      if (owner_.empty()) {
        requestControl(false);   // the next tick jogs, once the driver said yes
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 3000,
          "!!! %s has the control: joystick ignored (hold button %d for %.1f s)",
          owner_.c_str(), force_button_, force_hold_s_);
      }
    }
    return;
  }
  powerOnIfPending();
  if (gripper) {setGripper(*gripper);}

  // 3. Homing state transitions (click toggles, any jog input aborts)
  if (home_clicked) {
    if (homing_) {
      RCLCPP_INFO(get_logger(), ">>> homing cancelled");
      homing_ = false;
      stopArm();
    } else {
      RCLCPP_INFO(get_logger(), ">>> homing (click X again or touch a stick to abort)");
      homing_ = true;
      bad_reads_ = 0;
      active_.clear();
      hold_.clear();
    }
  }
  if (homing_ && !wanted.empty()) {
    RCLCPP_INFO(get_logger(), ">>> homing aborted by stick/bumper input");
    homing_ = false;
    stopArm();
  }

  // 4. Homing: bounded, synchronised steps toward all-zero
  if (homing_) {
    home();
    return;
  }

  // 5. Nothing requested -> stop once, then idle (no commands; the control lapses)
  if (wanted.empty()) {
    if (!active_.empty()) {
      RCLCPP_INFO(get_logger(), ">>> stop");
      stopArm();
    }
    hold_.clear();
    limit_reported_.clear();
    return;
  }

  // 6. Jogging requires fresh feedback
  const auto angles = freshAngles();
  if (!angles) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 1000, "!!! no fresh /joint_states, stopping");
    stopArm();
    return;
  }
  jog(wanted, *angles);
}

void JoystickTeleop::handleEvent(
  const input::Event & ev, bool & home_clicked, std::optional<bool> & gripper)
{
  const int64_t t_ms = ev.timeMs;
  const int number = ev.number;
  const int value = ev.value;

  if (ev.type == input::TYPE_AXIS) {
    if (tl::isTriggerAxis(number)) {return;}   // never a joint command
    axis_state_[number] = value;
    axis_changed_ms_[number] = t_ms;
    const bool near_speed_btn =
      tl::valueOr0(button_state_, tl::BUTTON_SPEED_MODE) == 1 ||
      (speed_btn_ms_ && std::llabs(t_ms - *speed_btn_ms_) <= tl::SPEED_BUTTON_GUARD_MS);
    if (near_speed_btn && std::abs(value) > tl::AXIS_DEADZONE) {
      if (!blocked_axes_.count(number) && debug_) {
        RCLCPP_INFO(get_logger(), "axis   %d blocked (fired with button 9)", number);
      }
      blocked_axes_.insert(number);
    } else if (blocked_axes_.count(number) && std::abs(value) <= tl::AXIS_DEADZONE) {
      blocked_axes_.erase(number);   // back at centre -> usable
      if (debug_) {RCLCPP_INFO(get_logger(), "axis   %d unblocked", number);}
    }
    if (debug_) {RCLCPP_INFO(get_logger(), "axis   %d = %d", number, value);}

  } else if (ev.type == input::TYPE_BUTTON) {
    button_state_[number] = value;
    if (debug_) {RCLCPP_INFO(get_logger(), "button %d = %d", number, value);}

    if (number == tl::BUTTON_SPEED_MODE) {
      speed_btn_ms_ = t_ms;
      // Block axes that moved just before button 9 fired
      for (const auto & [axis, axis_ms] : axis_changed_ms_) {
        if (t_ms - axis_ms <= tl::SPEED_BUTTON_GUARD_MS &&
          std::abs(tl::valueOr0(axis_state_, axis)) > tl::AXIS_DEADZONE)
        {
          blocked_axes_.insert(axis);
        }
      }
    }

    if (value == 1) {
      if (number == tl::BUTTON_GRIPPER_CLOSE) {
        gripper = true;
      } else if (number == tl::BUTTON_GRIPPER_OPEN) {
        gripper = false;
      } else if (number == tl::BUTTON_HOME) {
        home_clicked = true;
      } else if (number == tl::BUTTON_SPEED_MODE) {
        mode_index_ = (mode_index_ + 1) % tl::SPEED_MODES.size();
        setLed(mode());
        RCLCPP_INFO(
          get_logger(), ">>> speed mode: %s (cap %d %%, max coast %.0f deg)",
          mode().name, mode().maxSpeed, mode().leadMax);
      }
    }
  }
}

void JoystickTeleop::home()
{
  const auto angles = freshAngles();
  if (!angles) {
    ++bad_reads_;
    sendStop();
    if (bad_reads_ >= tl::HOME_MAX_BAD_READS) {
      RCLCPP_WARN(get_logger(), "!!! no valid angle feedback, homing aborted");
      homing_ = false;
    }
    return;
  }
  bad_reads_ = 0;

  const auto targets = tl::homeStep(*angles);
  if (!targets) {
    RCLCPP_INFO(get_logger(), ">>> home reached");
    homing_ = false;
    return;
  }
  ArmCommand cmd;
  cmd.type = ArmCommand::ALL_ANGLES;
  std::copy(targets->begin(), targets->end(), cmd.angles_deg.begin());
  cmd.speed = tl::HOME_SPEED;
  cmd.source = SOURCE;
  command_pub_->publish(cmd);
}

void JoystickTeleop::jog(const tl::Wanted & wanted, const mycobot::Angles & angles)
{
  const double now = nowSeconds();
  ArmCommand cmd;
  cmd.type = ArmCommand::JOINTS;
  cmd.source = SOURCE;
  auto add = [&cmd](int joint, double angle, int speed) {
      JointTarget t;
      t.joint = static_cast<uint8_t>(joint);
      t.angle_deg = tl::round1(angle);
      t.speed = static_cast<uint8_t>(speed);
      cmd.targets.push_back(t);
    };

  std::set<int> moving;
  for (const auto & [joint, motion] : wanted) {
    moving.insert(joint);
  }

  std::set<int> released;
  std::set_difference(
    active_.begin(), active_.end(), moving.begin(), moving.end(),
    std::inserter(released, released.begin()));

  std::set<int> still_reported;
  std::set_intersection(
    limit_reported_.begin(), limit_reported_.end(), moving.begin(), moving.end(),
    std::inserter(still_reported, still_reported.begin()));
  limit_reported_ = still_reported;

  // 6a. Latch idle joints (at session start or on release)
  for (int j = 1; j <= 6; ++j) {
    if (moving.count(j)) {
      hold_.erase(j);
    } else if (!hold_.count(j)) {
      hold_[j] = angles[j - 1];
    }
  }

  // 6b. Released joints: pin at the latched angle
  for (int j : released) {
    add(j, hold_[j], tl::HOLD_SPEED);
    last_hold_send_[j] = now;
  }

  // 6c. Idle joints that drifted (e.g. J4 sag while J6 rotates)
  for (const auto & [j, held] : hold_) {
    if (released.count(j)) {continue;}
    const double drift = angles[j - 1] - held;
    auto it = last_hold_send_.find(j);
    const double last_send = it == last_hold_send_.end() ? 0.0 : it->second;
    if (std::abs(drift) > tl::HOLD_TOLERANCE_DEG && now - last_send >= tl::HOLD_RESEND_PERIOD) {
      if (debug_) {
        RCLCPP_INFO(get_logger(), "hold J%d: drift %+.1f, back to %.1f", j, drift, held);
      }
      add(j, held, tl::HOLD_SPEED);
      last_hold_send_[j] = now;
    }
  }

  // 6d. Moving joints: proportional speed, slowed near limits
  for (const auto & [joint, motion] : wanted) {
    const double angle = angles[joint - 1];
    const auto shaped = tl::shapeNearLimit(
      mycobot::JOINT_LIMITS[joint - 1], angle,
      motion.direction, motion.speed, motion.lead);
    if (!shaped) {
      if (!limit_reported_.count(joint)) {
        RCLCPP_INFO(get_logger(), ">>> J%d at limit", joint);
        limit_reported_.insert(joint);
      }
      continue;
    }
    limit_reported_.erase(joint);
    if (debug_) {
      RCLCPP_INFO(
        get_logger(), "J%d %.1f -> %.1f @%d", joint, angle, shaped->target, shaped->speed);
    }
    add(joint, shaped->target, shaped->speed);
  }

  if (!cmd.targets.empty()) {command_pub_->publish(cmd);}

  if (moving != active_) {
    RCLCPP_INFO(get_logger(), ">>> jogging joints %s", fmtJoints(moving).c_str());
  }
  active_ = moving;
}

}  // namespace braco_teleop
