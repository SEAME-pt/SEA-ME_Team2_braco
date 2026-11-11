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

// move_to_pose - the mission node (issues #84, #85).
//
//   action  /move_to_pose     braco_interfaces/action/MoveToPose     point to point
//   action  /move_cartesian   braco_interfaces/action/MoveCartesian  straight lines
//   client  /arm/take_control (only with `use_control`, the real arm: ADR-0004)
//
// A goal is planned with MoveIt 2 (group `arm`, link `tcp`) and executed on arm_controller;
// then the tcp is read from TF and the error reported. The steps and their order are in
// mission.cpp and cartesian.cpp (no ROS, unit tested); this file only connects them to
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

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <cstdio>
#include <functional>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "braco_interfaces/action/move_cartesian.hpp"
#include "braco_interfaces/action/move_to_pose.hpp"
#include "braco_interfaces/srv/take_control.hpp"
#include "braco_mission/cartesian.hpp"
#include "braco_mission/mission.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/robot_state/robot_state.hpp"
#include "moveit/robot_trajectory/robot_trajectory.hpp"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/robot_state.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"

namespace
{

using MoveToPose = braco_interfaces::action::MoveToPose;
using GoalHandle = rclcpp_action::ServerGoalHandle<MoveToPose>;
using MoveCartesian = braco_interfaces::action::MoveCartesian;
using CartesianGoalHandle = rclcpp_action::ServerGoalHandle<MoveCartesian>;
using TakeControl = braco_interfaces::srv::TakeControl;
using moveit::planning_interface::MoveGroupInterface;
using namespace std::chrono_literals;

constexpr const char * SOURCE = "mission";   // control owner name (ADR-0002, ADR-0004)

braco_mission::Pose from_msg(const geometry_msgs::msg::Pose & p)
{
  return {{p.position.x, p.position.y, p.position.z},
    {p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w}};
}

geometry_msgs::msg::Pose to_msg(const braco_mission::Pose & p)
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

std::chrono::duration<double> seconds(double s)
{
  return std::chrono::duration<double>(s);
}

class MoveToPoseNode : public rclcpp::Node, public braco_mission::CartesianBackend
{
public:
  MoveToPoseNode()
  : Node("move_to_pose")
  {
    group_ = declare_parameter<std::string>("group", "arm");
    end_effector_ = declare_parameter<std::string>("end_effector", "tcp");
    settings_.use_control = declare_parameter<bool>("use_control", false);
    velocity_scaling_ = declare_parameter<double>("velocity_scaling", 0.1);
    acceleration_scaling_ = declare_parameter<double>("acceleration_scaling", 0.1);
    planning_time_s_ = declare_parameter<double>("planning_time_s", 5.0);
    settings_.position_tolerance_m = declare_parameter<double>("position_tolerance_m", 0.005);
    settings_.orientation_tolerance_rad =
      declare_parameter<double>("orientation_tolerance_deg", 2.0) * M_PI / 180.0;
    settle_s_ = declare_parameter<double>("settle_s", 0.5);
    tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 1.0);
    eef_step_m_ = declare_parameter<double>("eef_step_m", 0.005);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    control_client_ = create_client<TakeControl>("/arm/take_control");
  }

  // MoveGroupInterface needs the node spinning (it waits for the robot state): call it from
  // main once the executor runs.
  void start()
  {
    move_group_ = std::make_unique<MoveGroupInterface>(shared_from_this(), group_);
    move_group_->setEndEffectorLink(end_effector_);
    move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
    move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);
    move_group_->setPlanningTime(planning_time_s_);
    server_ = rclcpp_action::create_server<MoveToPose>(
      this, "move_to_pose",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveToPose::Goal>) {
        return accept();
      },
      [this](std::shared_ptr<GoalHandle>) {return cancel();},
      [this](std::shared_ptr<GoalHandle> goal) {
        std::thread([this, goal]() {run(goal);}).detach();
      });
    cartesian_server_ = rclcpp_action::create_server<MoveCartesian>(
      this, "move_cartesian",
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const MoveCartesian::Goal>) {
        return accept();
      },
      [this](std::shared_ptr<CartesianGoalHandle>) {return cancel();},
      [this](std::shared_ptr<CartesianGoalHandle> goal) {
        std::thread([this, goal]() {run_cartesian(goal);}).detach();
      });
    RCLCPP_INFO(
      get_logger(),
      "ready: /move_to_pose and /move_cartesian, group %s, link %s, planning frame %s, control %s",
      group_.c_str(), end_effector_.c_str(), move_group_->getPlanningFrame().c_str(),
      settings_.use_control ? "on (real arm)" : "off (simulation)");
  }

  // MoveGroupInterface holds this node (shared_from_this), so neither would ever be destroyed:
  // they would outlive main and class_loader would unload the kinematics plugin under a live
  // robot model (segfault on Ctrl+C). Break the cycle once the executor has stopped.
  void stop()
  {
    server_.reset();
    cartesian_server_.reset();
    move_group_.reset();
  }

  // ---- ArmBackend ----------------------------------------------------------------------

  bool take_control(std::string & message) override
  {
    auto response = call_take_control(false);
    if (!response) {
      message = "/arm/take_control did not answer";
      return false;
    }
    if (!response->success) {message = response->message + " (owner: " + response->owner + ")";}
    return response->success;
  }

  void release_control() override
  {
    if (!call_take_control(true)) {
      RCLCPP_WARN(get_logger(), "could not release the control: it frees itself after 5 s");
    }
  }

  bool plan(const braco_mission::Pose & goal, bool position_only, std::string & message) override
  {
    move_group_->setStartStateToCurrentState();
    move_group_->clearPoseTargets();
    if (position_only) {
      move_group_->setPositionTarget(
        goal.position.x, goal.position.y, goal.position.z, end_effector_);
    } else {
      move_group_->setPoseTarget(to_msg(goal), end_effector_);
    }
    const bool ok = static_cast<bool>(move_group_->plan(plan_));
    if (!ok) {message = position_only ? "no plan to the position" : "no plan to the pose";}
    return ok;
  }

  bool execute(std::string & message) override
  {
    const bool ok = static_cast<bool>(move_group_->execute(plan_));
    if (!ok) {message = "arm_controller did not complete the trajectory";}
    return ok;
  }

  std::optional<braco_mission::Pose> measure() override
  {
    std::this_thread::sleep_for(seconds(settle_s_));   // let the arm and TF settle
    try {
      const auto t = tf_buffer_->lookupTransform(
        move_group_->getPlanningFrame(), end_effector_, tf2::TimePointZero,
        tf2::durationFromSec(tf_timeout_s_));
      const auto & r = t.transform.rotation;
      const auto & p = t.transform.translation;
      return braco_mission::Pose{{p.x, p.y, p.z}, {r.x, r.y, r.z, r.w}};
    } catch (const tf2::TransformException & e) {
      RCLCPP_WARN(get_logger(), "tcp pose: %s", e.what());
      return std::nullopt;
    }
  }

  void report(const std::string & stage) override
  {
    if (feedback_) {feedback_(stage);}
    RCLCPP_INFO(get_logger(), "%s", stage.c_str());
  }

  // computeCartesianPath from the current state, then TOTG with the node's scaling: the
  // trajectory it returns is timed for full speed. The tcp of every point comes from forward
  // kinematics in the model frame, `world`, the planning frame of the waypoints.
  braco_mission::CartesianPlan plan_cartesian(
    const std::vector<braco_mission::Pose> & waypoints, double eef_step,
    std::string & message) override
  {
    braco_mission::CartesianPlan out;
    move_group_->setStartStateToCurrentState();
    std::vector<geometry_msgs::msg::Pose> poses;
    poses.reserve(waypoints.size());
    for (const auto & w : waypoints) {poses.push_back(to_msg(w));}

    moveit_msgs::msg::RobotTrajectory path;
    moveit_msgs::msg::MoveItErrorCodes error;
    out.fraction = move_group_->computeCartesianPath(poses, eef_step, path, true, &error);
    if (out.fraction < 0.0) {
      message = "computeCartesianPath failed, MoveIt error code " + std::to_string(error.val);
      return out;
    }

    const moveit::core::RobotStatePtr start = move_group_->getCurrentState(tf_timeout_s_);
    if (!start) {
      out.fraction = -1.0;
      message = "the current robot state is not available";
      return out;
    }
    robot_trajectory::RobotTrajectory trajectory(move_group_->getRobotModel(), group_);
    trajectory.setRobotTrajectoryMsg(*start, path);
    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(trajectory, velocity_scaling_, acceleration_scaling_)) {
      out.fraction = -1.0;
      message = "time parameterisation (TOTG) failed";
      return out;
    }

    for (std::size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
      moveit::core::RobotState state(trajectory.getWayPoint(i));
      state.updateLinkTransforms();
      const auto & tcp = state.getGlobalLinkTransform(end_effector_).translation();
      out.tcp_path.push_back({tcp.x(), tcp.y(), tcp.z()});
    }
    plan_.start_state = moveit_msgs::msg::RobotState();   // execute() sends the trajectory only
    trajectory.getRobotTrajectoryMsg(plan_.trajectory);
    RCLCPP_INFO(
      get_logger(), "Cartesian path: %.1f%% planned, %zu points, %.1f s",
      out.fraction * 100.0, trajectory.getWayPointCount(), trajectory.getDuration());
    return out;
  }

private:
  rclcpp_action::GoalResponse accept()
  {
    if (busy_.exchange(true)) {
      RCLCPP_WARN(get_logger(), "goal rejected: another goal is running");
      return rclcpp_action::GoalResponse::REJECT;
    }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse cancel()
  {
    move_group_->stop();
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  // Ends a goal of either action: a cancel stops the arm, so the steps report a failed
  // execution; say why it stopped.
  template<typename Handle, typename Result>
  void finish(const std::shared_ptr<Handle> & goal, const std::shared_ptr<Result> & result)
  {
    if (goal->is_canceling()) {
      result->success = false;
      result->message = "cancelled: " + result->message;
    }
    RCLCPP_INFO(get_logger(), "%s", result->message.c_str());
    if (goal->is_canceling()) {
      goal->canceled(result);
    } else if (result->success) {
      goal->succeed(result);
    } else {
      goal->abort(result);
    }
    feedback_ = nullptr;
    busy_ = false;
  }

  void run_cartesian(const std::shared_ptr<CartesianGoalHandle> & goal)
  {
    feedback_ = [goal](const std::string & stage) {
        auto feedback = std::make_shared<MoveCartesian::Feedback>();
        feedback->stage = stage;
        goal->publish_feedback(feedback);
      };
    auto result = std::make_shared<MoveCartesian::Result>();
    result->fraction = std::nan("");
    result->max_deviation_m = std::nan("");
    result->position_error_m = std::nan("");
    result->orientation_error_rad = std::nan("");
    const auto & request = *goal->get_goal();

    braco_mission::CartesianRequest cartesian;
    cartesian.eef_step_m = request.eef_step_m == 0.0 ? eef_step_m_ : request.eef_step_m;
    cartesian.min_fraction = request.min_fraction == 0.0 ? 1.0 : request.min_fraction;
    bool in_frame = true;
    for (const auto & w : request.waypoints) {
      geometry_msgs::msg::PoseStamped stamped;
      stamped.header = request.header;
      stamped.pose = w;
      std::optional<braco_mission::Pose> pose = in_planning_frame(stamped, result->message);
      if (!pose) {
        in_frame = false;
        break;
      }
      cartesian.waypoints.push_back(*pose);
    }

    if (in_frame) {
      const braco_mission::CartesianOutcome o =
        braco_mission::move_cartesian(*this, cartesian, settings_);
      result->success = o.success;
      result->fraction = o.fraction;
      result->max_deviation_m = o.max_deviation_m;
      result->position_error_m = o.error.position_m;
      result->orientation_error_rad = o.error.orientation_rad;
      result->message = o.message;
    }
    finish(goal, result);
  }

  void run(const std::shared_ptr<GoalHandle> & goal)
  {
    feedback_ = [goal](const std::string & stage) {
        auto feedback = std::make_shared<MoveToPose::Feedback>();
        feedback->stage = stage;
        goal->publish_feedback(feedback);
      };
    auto result = std::make_shared<MoveToPose::Result>();
    const auto & request = *goal->get_goal();

    std::optional<braco_mission::Pose> target = in_planning_frame(request.target, result->message);
    if (target) {
      const braco_mission::Outcome o =
        braco_mission::move_to_pose(*this, *target, request.position_only_fallback, settings_);
      result->success = o.success;
      result->position_error_m = o.error.position_m;
      result->orientation_error_rad = o.error.orientation_rad;
      result->position_only = o.position_only;
      result->message = o.message;
    } else {
      result->position_error_m = std::nan("");
      result->orientation_error_rad = std::nan("");
    }
    finish(goal, result);
  }

  // The goal in the planning frame. An empty frame_id means the planning frame already.
  std::optional<braco_mission::Pose> in_planning_frame(
    const geometry_msgs::msg::PoseStamped & target, std::string & message)
  {
    const std::string frame = move_group_->getPlanningFrame();
    if (target.header.frame_id.empty() || target.header.frame_id == frame) {
      return from_msg(target.pose);
    }
    try {
      geometry_msgs::msg::PoseStamped in_frame = target;
      in_frame.header.stamp = rclcpp::Time(0);   // the latest transform
      const auto out = tf_buffer_->transform(
        in_frame, frame, tf2::durationFromSec(tf_timeout_s_));
      // An orientation of all zeros means "position only": keep it so after the transform
      braco_mission::Pose pose = from_msg(out.pose);
      if (braco_mission::classify(from_msg(target.pose)) ==
        braco_mission::GoalKind::PositionOnly)
      {
        pose.orientation = {0.0, 0.0, 0.0, 0.0};
      }
      return pose;
    } catch (const tf2::TransformException & e) {
      message = "goal refused: no transform from " + target.header.frame_id + " to " + frame +
        ": " + e.what();
      return std::nullopt;
    }
  }

  TakeControl::Response::SharedPtr call_take_control(bool release)
  {
    if (!control_client_->wait_for_service(2s)) {return nullptr;}
    auto request = std::make_shared<TakeControl::Request>();
    request->source = SOURCE;
    request->force = false;   // ADR-0004: never take it from another client
    request->release = release;
    auto future = control_client_->async_send_request(request);
    if (future.wait_for(5s) != std::future_status::ready) {return nullptr;}
    return future.get();
  }

  std::string group_;
  std::string end_effector_;
  braco_mission::Settings settings_;
  double velocity_scaling_ = 0.1;
  double acceleration_scaling_ = 0.1;
  double planning_time_s_ = 5.0;
  double settle_s_ = 0.5;
  double tf_timeout_s_ = 1.0;
  double eef_step_m_ = 0.005;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Client<TakeControl>::SharedPtr control_client_;
  std::unique_ptr<MoveGroupInterface> move_group_;
  MoveGroupInterface::Plan plan_;
  rclcpp_action::Server<MoveToPose>::SharedPtr server_;
  rclcpp_action::Server<MoveCartesian>::SharedPtr cartesian_server_;
  // Feedback of the goal that runs (one at a time: busy_); set and used on its thread
  std::function<void(const std::string &)> feedback_;
  std::atomic<bool> busy_{false};
};

// The node's whole life. Its constructor and rclcpp may throw (a parameter of the wrong
// type, a failed init): main catches that, so nothing escapes it.
int run(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToPoseNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});
  int status = 0;
  try {
    node->start();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "%s", e.what());
    status = 1;
    rclcpp::shutdown();
  }
  spinner.join();
  node->stop();
  rclcpp::shutdown();
  return status;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    return run(argc, argv);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "move_to_pose: %s\n", e.what());
  } catch (...) {
    std::fprintf(stderr, "move_to_pose: unknown exception\n");
  }
  return 1;
}
