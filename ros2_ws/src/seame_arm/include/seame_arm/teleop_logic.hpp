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

// Joystick-to-joint logic of libs/cpp/srcs/joystick_all_joints.cpp, as pure
// functions (no I/O), so joystick_teleop can use it and tests can check it.
// Values and mapping are the same as in the stand-alone program.

#ifndef SEAME_ARM__TELEOP_LOGIC_HPP_
#define SEAME_ARM__TELEOP_LOGIC_HPP_

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <map>
#include <optional>
#include <set>

#include "mycobot.hpp"

namespace seame_arm
{
namespace teleop
{

// ---- Speed modes (speed units 1..100 = percent) ---------------------------
// maxSpeed:    jog speed at full stick deflection
// leadMax:     step-ahead at full deflection (= worst-case coast)
// bumperSpeed: fixed J6 speed for L1/R1
// r, g, b:     Atom LED colour
struct SpeedMode
{
  const char * name;
  int maxSpeed;
  double leadMax;
  int bumperSpeed;
  uint8_t r, g, b;
};

constexpr std::array<SpeedMode, 5> SPEED_MODES = {{
  {"15%", 15, 5.0, 10, 0, 0, 255},        // blue
  {"30%", 30, 10.0, 20, 0, 255, 0},       // green
  {"50%", 50, 15.0, 30, 128, 0, 255},     // purple
  {"75%", 75, 20.0, 40, 255, 255, 255},   // white
  {"100%", 100, 25.0, 50, 255, 0, 0},     // red
}};
constexpr size_t START_MODE = 1;     // 1 = medium

constexpr double MIN_SPEED = 3.0;    // speed just outside the deadzone
constexpr double STICK_EXPO = 1.8;   // >1 = more resolution near centre
constexpr int GRIPPER_SPEED = 50;

// ---- Step-ahead lead -------------------------------------------------------
constexpr double LEAD_MIN_DEG = 1.0;

// ---- Soft limit zone -------------------------------------------------------
constexpr double SLOW_ZONE_DEG = 25.0;
constexpr double MIN_LIMIT_FACTOR = 0.15;
constexpr double AT_LIMIT_DEG = 0.3;

// ---- One-click homing ------------------------------------------------------
constexpr int HOME_SPEED = 60;
constexpr double HOME_LEAD_DEG = 15.0;
constexpr double HOME_TOLERANCE_DEG = 1.0;
constexpr int HOME_MAX_BAD_READS = 3;

// ---- Idle-joint hold -------------------------------------------------------
constexpr double HOLD_TOLERANCE_DEG = 1.5;
constexpr double HOLD_RESEND_PERIOD = 0.3;   // seconds
constexpr int HOLD_SPEED = 10;

// ---- Input -----------------------------------------------------------------
constexpr int AXIS_DEADZONE = 5000;
constexpr double AXIS_MAX = 32767.0;
constexpr bool BUMPERS_EXCLUSIVE = true;

struct AxisMap
{
  int joint;
  int direction;
};

// axis number -> (joint 1..6, direction)
inline const std::map<int, AxisMap> & axisToJoint()
{
  static const std::map<int, AxisMap> map = {
    {0, {1, -1}},   // left stick X  -> joint 1 (base yaw) [inverted]
    {1, {2, 1}},    // left stick Y  -> joint 2 (shoulder pitch)
    {3, {4, -1}},   // right stick vertical   -> joint 4
    {2, {5, -1}},   // right stick horizontal -> joint 5
    {7, {3, 1}},    // d-pad Y       -> joint 3 (elbow)
  };
  return map;
}

// Analog trigger axes: ignored completely, never move a joint.
inline bool isTriggerAxis(int axis)
{
  return axis == 4 || axis == 5;
}

constexpr int BUTTON_GRIPPER_CLOSE = 0;   // A
constexpr int BUTTON_GRIPPER_OPEN = 1;    // B
constexpr int BUTTON_HOME = 3;            // X
constexpr int BUTTON_L1 = 6;              // J6 negative
constexpr int BUTTON_R1 = 7;              // J6 positive
constexpr int BUTTON_SPEED_MODE = 9;      // cycle speed mode

// Axes that change while button 9 is held, or within this window of its
// press/release, are ignored until they return to centre.
constexpr int64_t SPEED_BUTTON_GUARD_MS = 250;

// ---- Helpers ---------------------------------------------------------------

struct Motion
{
  int direction;
  double speed;
  double lead;
};
using Wanted = std::map<int, Motion>;   // joint -> motion (sorted by joint)

struct Shaped
{
  double target;
  int speed;
};

inline double round1(double v)
{
  return std::round(v * 10.0) / 10.0;
}

inline int valueOr0(const std::map<int, int> & m, int key)
{
  auto it = m.find(key);
  return it == m.end() ? 0 : it->second;
}

// Deflection beyond the deadzone, 0..1, shaped by the expo curve.
inline double stickRatio(int raw)
{
  const int magnitude = std::abs(raw);
  if (magnitude <= AXIS_DEADZONE) {return 0.0;}
  const double ratio = std::min(1.0, (magnitude - AXIS_DEADZONE) / (AXIS_MAX - AXIS_DEADZONE));
  return std::pow(ratio, STICK_EXPO);
}

// Map a 0..1 command ratio to (speed, lead) for the given mode.
inline Motion motionForRatio(double ratio, const SpeedMode & mode, int direction)
{
  return {direction,
    MIN_SPEED + ratio * (mode.maxSpeed - MIN_SPEED),
    LEAD_MIN_DEG + ratio * (mode.leadMax - LEAD_MIN_DEG)};
}

// Ratio that yields the mode's fixed J6 bumper speed.
inline double bumperRatio(const SpeedMode & mode)
{
  const double span = mode.maxSpeed - MIN_SPEED;
  return std::clamp((mode.bumperSpeed - MIN_SPEED) / span, 0.0, 1.0);
}

// Scale speed and lead by the distance left to the limit we move toward.
// Returns nothing if the joint is already at the limit.
inline std::optional<Shaped> shapeNearLimit(
  const mycobot::JointLimit & lim, double angle,
  int direction, double speed, double lead)
{
  const double remaining = direction > 0 ? (lim.max - angle) : (angle - lim.min);
  if (remaining <= AT_LIMIT_DEG) {return std::nullopt;}

  const double factor = remaining >= SLOW_ZONE_DEG ?
    1.0 :
    std::max(MIN_LIMIT_FACTOR, remaining / SLOW_ZONE_DEG);
  const int shapedSpeed = std::max(1, static_cast<int>(std::lround(speed * factor)));
  lead = std::min(lead * factor, remaining);   // never step past the limit
  return Shaped{angle + direction * lead, shapedSpeed};
}

// Desired motion per joint. Axes in `blocked` never produce motion.
inline Wanted wantedMotion(
  const std::map<int, int> & axisState,
  const std::map<int, int> & buttonState,
  const SpeedMode & mode,
  const std::set<int> & blocked)
{
  Wanted wanted;

  const bool l1 = valueOr0(buttonState, BUTTON_L1) == 1;
  const bool r1 = valueOr0(buttonState, BUTTON_R1) == 1;
  const int bumperDir = (l1 && !r1) ? -1 : ((r1 && !l1) ? 1 : 0);
  if (bumperDir != 0) {
    wanted[6] = motionForRatio(bumperRatio(mode), mode, bumperDir);
    if (BUMPERS_EXCLUSIVE) {return wanted;}   // ignore sticks while J6 rotates
  }

  for (const auto & [axis, map] : axisToJoint()) {
    if (blocked.count(axis)) {continue;}
    const int value = valueOr0(axisState, axis);
    const double ratio = stickRatio(value);
    if (ratio > 0.0) {
      const int sign = value > 0 ? map.direction : -map.direction;
      wanted[map.joint] = motionForRatio(ratio, mode, sign);
    }
  }
  return wanted;
}

// Homing step: the same fraction for every joint, so they all arrive together.
// Returns nothing when every joint is within HOME_TOLERANCE_DEG of zero.
inline std::optional<mycobot::Angles> homeStep(const mycobot::Angles & angles)
{
  double maxDev = 0.0;
  for (double a : angles) {
    maxDev = std::max(maxDev, std::abs(a));
  }
  if (maxDev <= HOME_TOLERANCE_DEG) {return std::nullopt;}

  const double scale = std::min(1.0, HOME_LEAD_DEG / maxDev);
  mycobot::Angles targets{};
  for (size_t i = 0; i < targets.size(); ++i) {
    targets[i] = round1(angles[i] * (1.0 - scale));
  }
  return targets;
}

}  // namespace teleop
}  // namespace seame_arm

#endif  // SEAME_ARM__TELEOP_LOGIC_HPP_
