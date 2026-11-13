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

// MoveToPoseNode (include/braco_mission/move_to_pose_node.hpp): MoveIt 2, TF and arm_driver.

#include "braco_mission/move_to_pose_node.hpp"

#include <optional>   // cpplint takes it for a C header: it must come first

#include <chrono>
#include <cmath>
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "braco_mission/moveit_motion_group.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

using namespace std::chrono_literals;

namespace braco_mission
{

namespace
{

constexpr const char * SOURCE = "mission";   // control owner name (ADR-0002, ADR-0004)

Pose from_msg(const geometry_msgs::msg::Pose & p)
{
  return {{p.position.x, p.position.y, p.position.z},
    {p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w}};
}

std::chrono::duration<double> seconds(double s)
{
  return std::chrono::duration<double>(s);
}

}  // namespace

MoveToPoseNode::MoveToPoseNode(
  const rclcpp::NodeOptions & options, MotionGroupFactory make_motion_group)
: Node("move_to_pose", options), make_motion_group_(std::move(make_motion_group))
{
  motion_config_.group = declare_parameter<std::string>("group", "arm");
  motion_config_.end_effector = declare_parameter<std::string>("end_effector", "tcp");
  settings_.use_control = declare_parameter<bool>("use_control", false);
  motion_config_.velocity_scaling = declare_parameter<double>("velocity_scaling", 0.1);
  motion_config_.acceleration_scaling = declare_parameter<double>("acceleration_scaling", 0.1);
  motion_config_.planning_time_s = declare_parameter<double>("planning_time_s", 5.0);
  settings_.position_tolerance_m = declare_parameter<double>("position_tolerance_m", 0.005);
  settings_.orientation_tolerance_rad =
    declare_parameter<double>("orientation_tolerance_deg", 2.0) * M_PI / 180.0;
  settle_s_ = declare_parameter<double>("settle_s", 0.5);
  tf_timeout_s_ = declare_parameter<double>("tf_timeout_s", 1.0);
  motion_config_.state_timeout_s = tf_timeout_s_;
  eef_step_m_ = declare_parameter<double>("eef_step_m", 0.005);

  if (!make_motion_group_) {
    make_motion_group_ = [](const rclcpp::Node::SharedPtr & node, const MotionConfig & config) {
        return std::make_unique<MoveItMotionGroup>(node, config);
      };
  }
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  control_client_ = create_client<TakeControl>("/arm/take_control");
}

void MoveToPoseNode::start()
{
  motion_group_ = make_motion_group_(shared_from_this(), motion_config_);
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
    motion_config_.group.c_str(), motion_config_.end_effector.c_str(),
    motion_group_->planning_frame().c_str(),
    settings_.use_control ? "on (real arm)" : "off (simulation)");
}

void MoveToPoseNode::stop()
{
  server_.reset();
  cartesian_server_.reset();
  motion_group_.reset();
}

// ---- ArmBackend ------------------------------------------------------------------------

bool MoveToPoseNode::take_control(std::string & message)
{
  auto response = call_take_control(false);
  if (!response) {
    message = "/arm/take_control did not answer";
    return false;
  }
  if (!response->success) {message = response->message + " (owner: " + response->owner + ")";}
  return response->success;
}

void MoveToPoseNode::release_control()
{
  if (!call_take_control(true)) {
    RCLCPP_WARN(get_logger(), "could not release the control: it frees itself after 5 s");
  }
}

bool MoveToPoseNode::plan(const Pose & goal, bool position_only, std::string & message)
{
  const bool ok = motion_group_->plan(goal, position_only);
  if (!ok) {message = position_only ? "no plan to the position" : "no plan to the pose";}
  return ok;
}

bool MoveToPoseNode::execute(std::string & message)
{
  const bool ok = motion_group_->execute();
  if (!ok) {message = "arm_controller did not complete the trajectory";}
  return ok;
}

std::optional<Pose> MoveToPoseNode::measure()
{
  std::this_thread::sleep_for(seconds(settle_s_));   // let the arm and TF settle
  try {
    const auto t = tf_buffer_->lookupTransform(
      motion_group_->planning_frame(), motion_config_.end_effector, tf2::TimePointZero,
      tf2::durationFromSec(tf_timeout_s_));
    const auto & r = t.transform.rotation;
    const auto & p = t.transform.translation;
    return Pose{{p.x, p.y, p.z}, {r.x, r.y, r.z, r.w}};
  } catch (const tf2::TransformException & e) {
    RCLCPP_WARN(get_logger(), "tcp pose: %s", e.what());
    return std::nullopt;
  }
}

void MoveToPoseNode::report(const std::string & stage)
{
  if (feedback_) {feedback_(stage);}
  RCLCPP_INFO(get_logger(), "%s", stage.c_str());
}

// ---- CartesianBackend ------------------------------------------------------------------

CartesianPlan MoveToPoseNode::plan_cartesian(
  const std::vector<Pose> & waypoints, double eef_step, std::string & message)
{
  return motion_group_->plan_cartesian(waypoints, eef_step, message);
}

// ---- private ---------------------------------------------------------------------------

rclcpp_action::GoalResponse MoveToPoseNode::accept()
{
  if (busy_.exchange(true)) {
    RCLCPP_WARN(get_logger(), "goal rejected: another goal is running");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveToPoseNode::cancel()
{
  motion_group_->stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

template<typename Handle, typename Result>
void MoveToPoseNode::finish(
  const std::shared_ptr<Handle> & goal, const std::shared_ptr<Result> & result)
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

void MoveToPoseNode::run_cartesian(const std::shared_ptr<CartesianGoalHandle> & goal)
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

  CartesianRequest cartesian;
  cartesian.eef_step_m = request.eef_step_m == 0.0 ? eef_step_m_ : request.eef_step_m;
  cartesian.min_fraction = request.min_fraction == 0.0 ? 1.0 : request.min_fraction;
  bool in_frame = true;
  for (const auto & w : request.waypoints) {
    geometry_msgs::msg::PoseStamped stamped;
    stamped.header = request.header;
    stamped.pose = w;
    std::optional<Pose> pose = in_planning_frame(stamped, result->message);
    if (!pose) {
      in_frame = false;
      break;
    }
    cartesian.waypoints.push_back(*pose);
  }

  if (in_frame) {
    const CartesianOutcome o = move_cartesian(*this, cartesian, settings_);
    result->success = o.success;
    result->fraction = o.fraction;
    result->max_deviation_m = o.max_deviation_m;
    result->position_error_m = o.error.position_m;
    result->orientation_error_rad = o.error.orientation_rad;
    result->message = o.message;
  }
  finish(goal, result);
}

void MoveToPoseNode::run(const std::shared_ptr<GoalHandle> & goal)
{
  feedback_ = [goal](const std::string & stage) {
      auto feedback = std::make_shared<MoveToPose::Feedback>();
      feedback->stage = stage;
      goal->publish_feedback(feedback);
    };
  auto result = std::make_shared<MoveToPose::Result>();
  const auto & request = *goal->get_goal();

  std::optional<Pose> target = in_planning_frame(request.target, result->message);
  if (target) {
    const Outcome o = move_to_pose(*this, *target, request.position_only_fallback, settings_);
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

std::optional<Pose> MoveToPoseNode::in_planning_frame(
  const geometry_msgs::msg::PoseStamped & target, std::string & message)
{
  const std::string frame = motion_group_->planning_frame();
  if (target.header.frame_id.empty() || target.header.frame_id == frame) {
    return from_msg(target.pose);
  }
  try {
    geometry_msgs::msg::PoseStamped in_frame = target;
    in_frame.header.stamp = rclcpp::Time(0);   // the latest transform
    const auto out = tf_buffer_->transform(
      in_frame, frame, tf2::durationFromSec(tf_timeout_s_));
    // An orientation of all zeros means "position only": keep it so after the transform
    Pose pose = from_msg(out.pose);
    if (classify(from_msg(target.pose)) == GoalKind::PositionOnly) {
      pose.orientation = {0.0, 0.0, 0.0, 0.0};
    }
    return pose;
  } catch (const tf2::TransformException & e) {
    message = "goal refused: no transform from " + target.header.frame_id + " to " + frame +
      ": " + e.what();
    return std::nullopt;
  }
}

MoveToPoseNode::TakeControl::Response::SharedPtr MoveToPoseNode::call_take_control(bool release)
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

}  // namespace braco_mission
