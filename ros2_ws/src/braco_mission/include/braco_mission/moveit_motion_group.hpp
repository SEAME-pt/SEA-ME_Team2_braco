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

// MoveItMotionGroup: MotionGroup with MoveIt 2 (MoveGroupInterface, computeCartesianPath, TOTG).
// Needs a running move_group: checked in Gazebo, not unit tested.

#ifndef BRACO_MISSION__MOVEIT_MOTION_GROUP_HPP_
#define BRACO_MISSION__MOVEIT_MOTION_GROUP_HPP_

#include <string>
#include <vector>

#include "braco_mission/motion_group.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"

namespace braco_mission
{

class MoveItMotionGroup : public MotionGroup
{
public:
  // `node` must be spinning: MoveGroupInterface waits for the robot state.
  MoveItMotionGroup(const rclcpp::Node::SharedPtr & node, const MotionConfig & config);

  std::string planning_frame() const override;
  bool plan(const Pose & goal, bool position_only) override;
  // computeCartesianPath from the current state, then TOTG with the scaling: the trajectory it
  // returns is timed for full speed. The tcp of every point comes from forward kinematics in
  // the model frame, `world`, the planning frame of the waypoints.
  CartesianPlan plan_cartesian(
    const std::vector<Pose> & waypoints, double eef_step, std::string & message) override;
  bool execute() override;
  void stop() override;

private:
  rclcpp::Logger logger_;
  MotionConfig config_;
  moveit::planning_interface::MoveGroupInterface group_;
  moveit::planning_interface::MoveGroupInterface::Plan plan_;
};

}  // namespace braco_mission

#endif  // BRACO_MISSION__MOVEIT_MOTION_GROUP_HPP_
