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

// Pose math of the mission: is a goal usable, and how far is the arm from it.
//
// No ROS: plain structs, so it is unit tested (test/test_pose_math.cpp). The node converts
// geometry_msgs/Pose to and from these.
//   classify   Invalid (not finite, or a non-zero orientation that is not a rotation),
//              PositionOnly (orientation all 0: "no orientation given"), FullPose
//   error      position distance (m) and rotation angle between the orientations (rad)

#ifndef BRACO_MISSION__POSE_MATH_HPP_
#define BRACO_MISSION__POSE_MATH_HPP_

#include <algorithm>
#include <cmath>
#include <iterator>
#include <limits>

namespace braco_mission
{

struct Vec3
{
  double x, y, z;
};

struct Quat
{
  double x, y, z, w;
};

struct Pose
{
  Vec3 position;
  Quat orientation;
};

struct PoseError
{
  double position_m;
  double orientation_rad;   // NaN when the orientation was not compared
};

enum class GoalKind { Invalid, PositionOnly, FullPose };

// A quaternion this close to unit length is a rotation; a goal from the command line is
// rarely exact, so it is normalized, but one far from unit length is a typo.
constexpr double QUAT_NORM_TOLERANCE = 1e-3;

inline double norm(const Quat & q)
{
  return std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
}

inline Quat normalized(const Quat & q)
{
  const double n = norm(q);
  return {q.x / n, q.y / n, q.z / n, q.w / n};
}

inline bool is_finite(const Pose & p)
{
  const double v[] = {p.position.x, p.position.y, p.position.z,
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w};
  return std::all_of(std::begin(v), std::end(v), [](double d) {return std::isfinite(d);});
}

inline GoalKind classify(const Pose & goal)
{
  if (!is_finite(goal)) {return GoalKind::Invalid;}
  const double n = norm(goal.orientation);
  if (n == 0.0) {return GoalKind::PositionOnly;}
  if (std::abs(n - 1.0) > QUAT_NORM_TOLERANCE) {return GoalKind::Invalid;}
  return GoalKind::FullPose;
}

inline double position_error(const Vec3 & a, const Vec3 & b)
{
  return std::sqrt(
    (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) + (a.z - b.z) * (a.z - b.z));
}

// Angle of the rotation from a to b, in [0, pi]. q and -q are the same rotation: |dot| makes
// them 0 apart. Both are normalized first.
inline double orientation_error(const Quat & a, const Quat & b)
{
  const Quat qa = normalized(a);
  const Quat qb = normalized(b);
  const double dot = std::abs(qa.x * qb.x + qa.y * qb.y + qa.z * qb.z + qa.w * qb.w);
  return 2.0 * std::acos(std::min(1.0, dot));
}

inline PoseError pose_error(const Pose & goal, const Pose & measured, bool compare_orientation)
{
  return {
    position_error(goal.position, measured.position),
    compare_orientation ? orientation_error(goal.orientation, measured.orientation) :
    std::numeric_limits<double>::quiet_NaN()};
}

// Within both tolerances; an orientation that was not compared (NaN) is not checked.
inline bool within(
  const PoseError & e, double position_tolerance_m, double orientation_tolerance_rad)
{
  if (!(e.position_m <= position_tolerance_m)) {return false;}
  return std::isnan(e.orientation_rad) || e.orientation_rad <= orientation_tolerance_rad;
}

}  // namespace braco_mission

#endif  // BRACO_MISSION__POSE_MATH_HPP_
