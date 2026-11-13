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

// MoveToPoseNode - the mission node (issues #84, #85).
//
//   action  /move_to_pose     braco_interfaces/action/MoveToPose     point to point
//   action  /move_cartesian   braco_interfaces/action/MoveCartesian  straight lines
//   client  /arm/take_control (only with `use_control`, the real arm: ADR-0004)
//
// A goal is planned with MoveIt 2 (group `arm`, link `tcp`) and executed on arm_controller;
// then the tcp is read from TF and the error reported. The steps and their order are in
// mission.cpp and cartesian.cpp (no ROS, unit tested); this node only connects them to
// MoveIt 2, TF and arm_driver. One goal at a time, on either action: another one is rejected
// while the first runs.
//
// A straight-line path comes from computeCartesianPath, then is time-parameterised (TOTG)
// with velocity_scaling / acceleration_scaling, so it never runs faster than ADR-0004 allows.
//
// Parameters: group (arm), end_effector (tcp), use_control (false: simulation),
// velocity_scaling / acceleration_scaling (0.1, ADR-0004), planning_time_s (5.0),
// position_tolerance_m (0.005), orientation_tolerance_deg (2.0), settle_s (0.5),
// tf_timeout_s (1.0), eef_step_m (0.005: the default of a MoveCartesian goal).
//
// In a library, so tests can run it in-process; src/move_to_pose.cpp only holds main().
// MoveIt 2 is behind MotionGroup: tests pass a fake through the factory.

#ifndef BRACO_MISSION__MOVE_TO_POSE_NODE_HPP_
#define BRACO_MISSION__MOVE_TO_POSE_NODE_HPP_

#include <optional>   // cpplint takes it for a C header: it must come first

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "braco_interfaces/action/move_cartesian.hpp"
#include "braco_interfaces/action/move_to_pose.hpp"
#include "braco_interfaces/srv/take_control.hpp"
#include "braco_mission/cartesian.hpp"
#include "braco_mission/mission.hpp"
#include "braco_mission/motion_group.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace braco_mission
{

class MoveToPoseNode : public rclcpp::Node, public CartesianBackend
{
public:
  using MoveToPose = braco_interfaces::action::MoveToPose;
  using GoalHandle = rclcpp_action::ServerGoalHandle<MoveToPose>;
  using MoveCartesian = braco_interfaces::action::MoveCartesian;
  using CartesianGoalHandle = rclcpp_action::ServerGoalHandle<MoveCartesian>;
  using TakeControl = braco_interfaces::srv::TakeControl;
  // Builds the MotionGroup in start(); empty: MoveItMotionGroup.
  using MotionGroupFactory = std::function<std::unique_ptr<MotionGroup>(
        const rclcpp::Node::SharedPtr &, const MotionConfig &)>;

  explicit MoveToPoseNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions(),
    MotionGroupFactory make_motion_group = nullptr);

  // MoveGroupInterface needs the node spinning (it waits for the robot state): call it from
  // main once the executor runs.
  void start();

  // MoveGroupInterface holds this node (shared_from_this), so neither would ever be destroyed:
  // they would outlive main and class_loader would unload the kinematics plugin under a live
  // robot model (segfault on Ctrl+C). Break the cycle once the executor has stopped.
  void stop();

  // ---- ArmBackend ----------------------------------------------------------------------

  bool take_control(std::string & message) override;
  void release_control() override;
  bool plan(const Pose & goal, bool position_only, std::string & message) override;
  bool execute(std::string & message) override;
  std::optional<Pose> measure() override;
  void report(const std::string & stage) override;

  // ---- CartesianBackend ----------------------------------------------------------------

  CartesianPlan plan_cartesian(
    const std::vector<Pose> & waypoints, double eef_step, std::string & message) override;

private:
  rclcpp_action::GoalResponse accept();
  rclcpp_action::CancelResponse cancel();

  // Ends a goal of either action: a cancel stops the arm, so the steps report a failed
  // execution; say why it stopped.
  template<typename Handle, typename Result>
  void finish(const std::shared_ptr<Handle> & goal, const std::shared_ptr<Result> & result);

  void run_cartesian(const std::shared_ptr<CartesianGoalHandle> & goal);
  void run(const std::shared_ptr<GoalHandle> & goal);

  // The goal in the planning frame. An empty frame_id means the planning frame already.
  std::optional<Pose> in_planning_frame(
    const geometry_msgs::msg::PoseStamped & target, std::string & message);

  TakeControl::Response::SharedPtr call_take_control(bool release);

  MotionConfig motion_config_;
  Settings settings_;
  double settle_s_ = 0.5;
  double tf_timeout_s_ = 1.0;
  double eef_step_m_ = 0.005;

  MotionGroupFactory make_motion_group_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<TakeControl>::SharedPtr control_client_;
  std::unique_ptr<MotionGroup> motion_group_;
  rclcpp_action::Server<MoveToPose>::SharedPtr server_;
  rclcpp_action::Server<MoveCartesian>::SharedPtr cartesian_server_;
  // Feedback of the goal that runs (one at a time: busy_); set and used on its thread
  std::function<void(const std::string &)> feedback_;
  std::atomic<bool> busy_{false};
};

}  // namespace braco_mission

#endif  // BRACO_MISSION__MOVE_TO_POSE_NODE_HPP_
