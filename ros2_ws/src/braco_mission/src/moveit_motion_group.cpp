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

#include "braco_mission/moveit_motion_group.hpp"

#include <cstddef>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose.hpp"
#include "moveit/robot_state/robot_state.hpp"
#include "moveit/robot_trajectory/robot_trajectory.hpp"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/robot_state.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"

namespace braco_mission
{

namespace
{

geometry_msgs::msg::Pose to_msg(const Pose & p)
{
  geometry_msgs::msg::Pose m;
  m.position.x = p.position.x;
  m.position.y = p.position.y;
  m.position.z = p.position.z;
  m.orientation.x = p.orientation.x;
  m.orientation.y = p.orientation.y;
  m.orientation.z = p.orientation.z;
  m.orientation.w = p.orientation.w;
  return m;
}

}  // namespace

MoveItMotionGroup::MoveItMotionGroup(
  const rclcpp::Node::SharedPtr & node, const MotionConfig & config)
: logger_(node->get_logger()), config_(config), group_(node, config.group)
{
  group_.setEndEffectorLink(config_.end_effector);
  group_.setMaxVelocityScalingFactor(config_.velocity_scaling);
  group_.setMaxAccelerationScalingFactor(config_.acceleration_scaling);
  group_.setPlanningTime(config_.planning_time_s);
}

std::string MoveItMotionGroup::planning_frame() const
{
  return group_.getPlanningFrame();
}

bool MoveItMotionGroup::plan(const Pose & goal, bool position_only)
{
  group_.setStartStateToCurrentState();
  group_.clearPoseTargets();
  if (position_only) {
    group_.setPositionTarget(
      goal.position.x, goal.position.y, goal.position.z, config_.end_effector);
  } else {
    group_.setPoseTarget(to_msg(goal), config_.end_effector);
  }
  return static_cast<bool>(group_.plan(plan_));
}

CartesianPlan MoveItMotionGroup::plan_cartesian(
  const std::vector<Pose> & waypoints, double eef_step, std::string & message)
{
  CartesianPlan out;
  group_.setStartStateToCurrentState();
  std::vector<geometry_msgs::msg::Pose> poses;
  poses.reserve(waypoints.size());
  for (const auto & w : waypoints) {
    poses.push_back(to_msg(w));
  }

  moveit_msgs::msg::RobotTrajectory path;
  moveit_msgs::msg::MoveItErrorCodes error;
  out.fraction = group_.computeCartesianPath(poses, eef_step, path, true, &error);
  if (out.fraction < 0.0) {
    message = "computeCartesianPath failed, MoveIt error code " + std::to_string(error.val);
    return out;
  }

  const moveit::core::RobotStatePtr start_state = group_.getCurrentState(config_.state_timeout_s);
  if (!start_state) {
    out.fraction = -1.0;
    message = "the current robot state is not available";
    return out;
  }
  robot_trajectory::RobotTrajectory trajectory(group_.getRobotModel(), config_.group);
  trajectory.setRobotTrajectoryMsg(*start_state, path);
  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(
      trajectory, config_.velocity_scaling, config_.acceleration_scaling))
  {
    out.fraction = -1.0;
    message = "time parameterisation (TOTG) failed";
    return out;
  }

  for (std::size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
    moveit::core::RobotState state(trajectory.getWayPoint(i));
    state.updateLinkTransforms();
    const auto & tcp = state.getGlobalLinkTransform(config_.end_effector).translation();
    out.tcp_path.push_back({tcp.x(), tcp.y(), tcp.z()});
  }
  plan_.start_state = moveit_msgs::msg::RobotState();   // execute() sends the trajectory only
  trajectory.getRobotTrajectoryMsg(plan_.trajectory);
  RCLCPP_INFO(
    logger_, "Cartesian path: %.1f%% planned, %zu points, %.1f s",
    out.fraction * 100.0, trajectory.getWayPointCount(), trajectory.getDuration());
  return out;
}

bool MoveItMotionGroup::execute()
{
  return static_cast<bool>(group_.execute(plan_));
}

void MoveItMotionGroup::stop()
{
  group_.stop();
}

}  // namespace braco_mission
