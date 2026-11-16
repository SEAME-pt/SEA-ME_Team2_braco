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

// MoveToPoseNode with a fake MotionGroup: a test node in the same executor sends the action
// goals, serves /arm/take_control and publishes static TF (world -> tcp, world -> table).

#include <gtest/gtest.h>
#include <sys/wait.h>

#include <optional>   // cpplint takes it for a C header: it must come first

#include <atomic>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdlib>
#include <future>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "braco_mission/move_to_pose_node.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"

using namespace std::chrono_literals;
using braco_interfaces::srv::TakeControl;
using braco_mission::CartesianPlan;
using braco_mission::MotionConfig;
using braco_mission::MotionGroup;
using braco_mission::MoveToPoseNode;
using braco_mission::Pose;
using MoveToPose = braco_interfaces::action::MoveToPose;
using MoveCartesian = braco_interfaces::action::MoveCartesian;
using rclcpp_action::ResultCode;

namespace
{

const Pose TCP{{0.15, 0.0, 0.25}, {0.0, 0.0, 0.0, 1.0}};   // world -> tcp
constexpr double TABLE_X = 1.0;                              // world -> table

bool contains(const std::string & text, const std::string & part)
{
  return text.find(part) != std::string::npos;
}

// Records every call; plans and executes unless told otherwise. With `hold`, execute() waits
// for stop() or release().
class FakeMotionGroup : public MotionGroup
{
public:
  std::atomic<bool> plans{true};
  std::atomic<bool> executes{true};
  std::atomic<bool> hold{false};
  std::atomic<double> fraction{1.0};
  std::atomic<int> stops{0};

  std::string planning_frame() const override {return "world";}

  bool plan(const Pose & goal, bool position_only) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    planned_.push_back(goal);
    position_only_.push_back(position_only);
    return plans;
  }

  CartesianPlan plan_cartesian(
    const std::vector<Pose> & waypoints, double eef_step, std::string & message) override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    eef_step_ = eef_step;
    planned_.insert(planned_.end(), waypoints.begin(), waypoints.end());
    CartesianPlan out;
    out.fraction = fraction;
    if (out.fraction < 0.0) {
      message = "TOTG failed";
    } else {
      out.tcp_path = {TCP.position, waypoints.back().position};
    }
    return out;
  }

  bool execute() override
  {
    std::unique_lock<std::mutex> lock(mutex_);
    ++executions_;
    cv_.notify_all();
    cv_.wait(lock, [this] {return !hold;});
    return executes && !stopped_;
  }

  void stop() override
  {
    std::lock_guard<std::mutex> lock(mutex_);
    ++stops;
    stopped_ = true;
    hold = false;
    cv_.notify_all();
  }

  void release()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    hold = false;
    cv_.notify_all();
  }

  bool wait_for_execute()
  {
    std::unique_lock<std::mutex> lock(mutex_);
    return cv_.wait_for(lock, 5s, [this] {return executions_ > 0;});
  }

  std::vector<Pose> planned()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return planned_;
  }
  std::vector<bool> position_only()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return position_only_;
  }
  double eef_step()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return eef_step_;
  }
  int executions()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return executions_;
  }

private:
  std::mutex mutex_;
  std::condition_variable cv_;
  std::vector<Pose> planned_;
  std::vector<bool> position_only_;
  double eef_step_ = 0.0;
  int executions_ = 0;
  bool stopped_ = false;
};

geometry_msgs::msg::TransformStamped world_to(
  const rclcpp::Time & stamp, const std::string & child, double x, double y, double z)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = stamp;
  t.header.frame_id = "world";
  t.child_frame_id = child;
  t.transform.translation.x = x;
  t.transform.translation.y = y;
  t.transform.translation.z = z;
  t.transform.rotation.w = 1.0;
  return t;
}

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

MoveToPose::Goal pose_goal(const std::string & frame, const Pose & pose)
{
  MoveToPose::Goal goal;
  goal.target.header.frame_id = frame;
  goal.target.pose = to_msg(pose);
  return goal;
}

MoveCartesian::Goal line_goal(const std::string & frame, double eef_step = 0.0)
{
  MoveCartesian::Goal goal;
  goal.header.frame_id = frame;
  goal.waypoints.push_back(to_msg(TCP));
  goal.eef_step_m = eef_step;
  return goal;
}

class MoveToPoseNodeTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void TearDown() override
  {
    if (fake_) {fake_->release();}
    std::this_thread::sleep_for(100ms);   // a goal thread ends after its result is sent
    if (executor_) {executor_->cancel();}
    if (spinner_.joinable()) {spinner_.join();}
    if (node_) {node_->stop();}   // as main does once the executor has stopped
    helper_.reset();
    node_.reset();
  }

  // The node (fast settle and TF timeout) with a fake MotionGroup, and the test node, spinning.
  // `granted`: how /arm/take_control answers; nothing: no such service.
  void start(
    const std::vector<rclcpp::Parameter> & extra = {}, std::optional<bool> granted = std::nullopt)
  {
    helper_ = std::make_shared<rclcpp::Node>("move_to_pose_test");
    tf_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(helper_);
    tf_->sendTransform(
      {world_to(helper_->now(), "tcp", TCP.position.x, TCP.position.y, TCP.position.z),
        world_to(helper_->now(), "table", TABLE_X, 0.0, 0.0)});
    if (granted) {
      control_ = helper_->create_service<TakeControl>(
        "/arm/take_control",
        [this, ok = *granted](
          const TakeControl::Request::SharedPtr request,
          TakeControl::Response::SharedPtr response) {
          std::lock_guard<std::mutex> lock(mutex_);
          control_requests_.push_back(*request);
          response->success = ok || request->release;
          response->owner = ok ? request->source : "web";
          response->message = ok ? "" : "owned by web";
        });
    }
    pose_client_ = rclcpp_action::create_client<MoveToPose>(helper_, "move_to_pose");
    line_client_ = rclcpp_action::create_client<MoveCartesian>(helper_, "move_cartesian");

    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("settle_s", 0.0), rclcpp::Parameter("tf_timeout_s", 0.5)};
    params.insert(params.end(), extra.begin(), extra.end());
    node_ = std::make_shared<MoveToPoseNode>(
      rclcpp::NodeOptions().parameter_overrides(params),
      [this](const rclcpp::Node::SharedPtr &, const MotionConfig &) {
        auto fake = std::make_unique<FakeMotionGroup>();
        fake_ = fake.get();
        return fake;
      });
    node_->start();

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(helper_);
    spinner_ = std::thread([this] {executor_->spin();});
    ASSERT_TRUE(pose_client_->wait_for_action_server(5s));
    ASSERT_TRUE(line_client_->wait_for_action_server(5s));
    std::this_thread::sleep_for(200ms);   // the node's TF listener gets the static frames
  }

  template<typename Action>
  typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr send(
    const typename rclcpp_action::Client<Action>::SharedPtr & client,
    const typename Action::Goal & goal)
  {
    auto future = client->async_send_goal(goal);
    if (future.wait_for(5s) != std::future_status::ready) {return nullptr;}
    return future.get();
  }

  template<typename Action>
  typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult result(
    const typename rclcpp_action::Client<Action>::SharedPtr & client,
    const typename rclcpp_action::ClientGoalHandle<Action>::SharedPtr & handle)
  {
    typename rclcpp_action::ClientGoalHandle<Action>::WrappedResult out{};
    out.code = ResultCode::UNKNOWN;
    out.result = std::make_shared<typename Action::Result>();
    if (!handle) {
      ADD_FAILURE() << "goal rejected";
      return out;
    }
    auto future = client->async_get_result(handle);
    if (future.wait_for(10s) != std::future_status::ready) {
      ADD_FAILURE() << "no result";
      return out;
    }
    return future.get();
  }

  rclcpp_action::ClientGoalHandle<MoveToPose>::WrappedResult move(const MoveToPose::Goal & goal)
  {
    return result<MoveToPose>(pose_client_, send<MoveToPose>(pose_client_, goal));
  }

  rclcpp_action::ClientGoalHandle<MoveCartesian>::WrappedResult line(
    const MoveCartesian::Goal & goal)
  {
    return result<MoveCartesian>(line_client_, send<MoveCartesian>(line_client_, goal));
  }

  std::vector<TakeControl::Request> control_requests()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return control_requests_;
  }

  FakeMotionGroup * fake_ = nullptr;   // owned by node_
  std::shared_ptr<MoveToPoseNode> node_;
  rclcpp::Node::SharedPtr helper_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_;
  rclcpp::Service<TakeControl>::SharedPtr control_;
  rclcpp_action::Client<MoveToPose>::SharedPtr pose_client_;
  rclcpp_action::Client<MoveCartesian>::SharedPtr line_client_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spinner_;
  std::mutex mutex_;
  std::vector<TakeControl::Request> control_requests_;
};

}  // namespace

// ---- the node ----------------------------------------------------------------------------

TEST_F(MoveToPoseNodeTest, DeclaresTheDefaultParameters)
{
  const auto node = std::make_shared<MoveToPoseNode>();   // MoveIt: not started, not reached
  EXPECT_EQ(node->get_parameter("group").as_string(), "arm");
  EXPECT_EQ(node->get_parameter("end_effector").as_string(), "tcp");
  EXPECT_FALSE(node->get_parameter("use_control").as_bool());
  EXPECT_DOUBLE_EQ(node->get_parameter("velocity_scaling").as_double(), 0.1);
  EXPECT_DOUBLE_EQ(node->get_parameter("acceleration_scaling").as_double(), 0.1);
}

TEST_F(MoveToPoseNodeTest, ProgramExitsOnABadParameter)
{
  const std::string command = std::string(MOVE_TO_POSE_EXE) +
    " --ros-args -p settle_s:=oops > /dev/null 2>&1";
  const int status = std::system(command.c_str());
  ASSERT_TRUE(WIFEXITED(status));
  EXPECT_EQ(WEXITSTATUS(status), 1);
}

// ---- /move_to_pose ---------------------------------------------------------------------

TEST_F(MoveToPoseNodeTest, ReachesAPoseInThePlanningFrame)
{
  start();
  const auto r = move(pose_goal("", TCP));
  EXPECT_EQ(r.code, ResultCode::SUCCEEDED);
  EXPECT_TRUE(r.result->success);
  EXPECT_TRUE(contains(r.result->message, "reached"));
  EXPECT_NEAR(r.result->position_error_m, 0.0, 1e-9);
  ASSERT_EQ(fake_->planned().size(), 1u);
  EXPECT_FALSE(fake_->position_only().front());
}

TEST_F(MoveToPoseNodeTest, TransformsTheGoalIntoThePlanningFrame)
{
  start();
  Pose in_table = TCP;
  in_table.position.x -= TABLE_X;
  const auto r = move(pose_goal("table", in_table));
  EXPECT_TRUE(r.result->success) << r.result->message;
  ASSERT_EQ(fake_->planned().size(), 1u);
  EXPECT_NEAR(fake_->planned().front().position.x, TCP.position.x, 1e-9);
}

TEST_F(MoveToPoseNodeTest, PositionOnlyGoalStaysPositionOnlyAfterTheTransform)
{
  start();
  Pose in_table{{TCP.position.x - TABLE_X, 0.0, TCP.position.z}, {0.0, 0.0, 0.0, 0.0}};
  const auto r = move(pose_goal("table", in_table));
  EXPECT_TRUE(r.result->success) << r.result->message;
  EXPECT_TRUE(r.result->position_only);
  EXPECT_TRUE(std::isnan(r.result->orientation_error_rad));
  ASSERT_EQ(fake_->position_only().size(), 1u);
  EXPECT_TRUE(fake_->position_only().front());
}

TEST_F(MoveToPoseNodeTest, UnknownFrameIsRefused)
{
  start();
  const auto r = move(pose_goal("nowhere", TCP));
  EXPECT_EQ(r.code, ResultCode::ABORTED);
  EXPECT_TRUE(contains(r.result->message, "goal refused: no transform from nowhere to world"));
  EXPECT_TRUE(std::isnan(r.result->position_error_m));
  EXPECT_TRUE(fake_->planned().empty());
}

TEST_F(MoveToPoseNodeTest, NoPlanAborts)
{
  start();
  fake_->plans = false;
  const auto r = move(pose_goal("", TCP));
  EXPECT_EQ(r.code, ResultCode::ABORTED);
  EXPECT_TRUE(contains(r.result->message, "no plan: no plan to the pose"));
  EXPECT_EQ(fake_->executions(), 0);
}

TEST_F(MoveToPoseNodeTest, FailedExecutionAborts)
{
  start();
  fake_->executes = false;
  const auto r = move(pose_goal("", TCP));
  EXPECT_EQ(r.code, ResultCode::ABORTED);
  EXPECT_TRUE(
    contains(r.result->message, "execution failed: arm_controller did not complete"));
}

TEST_F(MoveToPoseNodeTest, NoTcpTransformIsAFailure)
{
  start({rclcpp::Parameter("end_effector", "nowhere")});
  const auto r = move(pose_goal("", TCP));
  EXPECT_EQ(r.code, ResultCode::ABORTED);
  EXPECT_TRUE(contains(r.result->message, "not available from TF"));
}

TEST_F(MoveToPoseNodeTest, CancelStopsTheArm)
{
  start();
  fake_->hold = true;
  const auto handle = send<MoveToPose>(pose_client_, pose_goal("", TCP));
  ASSERT_TRUE(handle);
  ASSERT_TRUE(fake_->wait_for_execute());
  auto cancel = pose_client_->async_cancel_goal(handle);
  ASSERT_EQ(cancel.wait_for(5s), std::future_status::ready);
  const auto r = result<MoveToPose>(pose_client_, handle);
  EXPECT_EQ(r.code, ResultCode::CANCELED);
  EXPECT_FALSE(r.result->success);
  EXPECT_EQ(r.result->message.rfind("cancelled: ", 0), 0u);
  EXPECT_EQ(fake_->stops, 1);
}

TEST_F(MoveToPoseNodeTest, OneGoalAtATimeOnEitherAction)
{
  start();
  fake_->hold = true;
  const auto first = send<MoveToPose>(pose_client_, pose_goal("", TCP));
  ASSERT_TRUE(first);
  ASSERT_TRUE(fake_->wait_for_execute());
  EXPECT_FALSE(send<MoveToPose>(pose_client_, pose_goal("", TCP)));
  EXPECT_FALSE(send<MoveCartesian>(line_client_, line_goal("")));
  fake_->release();
  EXPECT_EQ(result<MoveToPose>(pose_client_, first).code, ResultCode::SUCCEEDED);
}

// ---- /arm/take_control (use_control: the real arm) -------------------------------------

TEST_F(MoveToPoseNodeTest, RealArmTakesAndReleasesTheControl)
{
  start({rclcpp::Parameter("use_control", true)}, true);
  const auto r = move(pose_goal("", TCP));
  EXPECT_TRUE(r.result->success) << r.result->message;
  const auto calls = control_requests();
  ASSERT_EQ(calls.size(), 2u);
  EXPECT_EQ(calls[0].source, "mission");
  EXPECT_FALSE(calls[0].force);
  EXPECT_FALSE(calls[0].release);
  EXPECT_TRUE(calls[1].release);
}

TEST_F(MoveToPoseNodeTest, RefusedControlSaysWhoOwnsIt)
{
  start({rclcpp::Parameter("use_control", true)}, false);
  const auto r = move(pose_goal("", TCP));
  EXPECT_EQ(r.code, ResultCode::ABORTED);
  EXPECT_TRUE(contains(r.result->message, "control refused: owned by web (owner: web)"));
  EXPECT_TRUE(fake_->planned().empty());
}

TEST_F(MoveToPoseNodeTest, NoControlServiceIsARefusal)
{
  start({rclcpp::Parameter("use_control", true)});
  const auto r = move(pose_goal("", TCP));
  EXPECT_EQ(r.code, ResultCode::ABORTED);
  EXPECT_TRUE(contains(r.result->message, "/arm/take_control did not answer"));
}

// ---- /move_cartesian -------------------------------------------------------------------

TEST_F(MoveToPoseNodeTest, LineReachesTheWaypointWithTheDefaultStep)
{
  start();
  const auto r = line(line_goal(""));
  EXPECT_EQ(r.code, ResultCode::SUCCEEDED) << r.result->message;
  EXPECT_DOUBLE_EQ(r.result->fraction, 1.0);
  EXPECT_DOUBLE_EQ(fake_->eef_step(), 0.005);   // eef_step_m 0: the node's eef_step_m
}

TEST_F(MoveToPoseNodeTest, LineUsesTheGoalStep)
{
  start();
  line(line_goal("", 0.01));
  EXPECT_DOUBLE_EQ(fake_->eef_step(), 0.01);
}

TEST_F(MoveToPoseNodeTest, PartialLineIsNotExecuted)
{
  start();
  fake_->fraction = 0.5;
  const auto r = line(line_goal(""));
  EXPECT_EQ(r.code, ResultCode::ABORTED);
  EXPECT_TRUE(contains(r.result->message, "(minimum 100.0%)"));   // min_fraction 0: 1.0
  EXPECT_EQ(fake_->executions(), 0);
}

TEST_F(MoveToPoseNodeTest, LinePlanningErrorAborts)
{
  start();
  fake_->fraction = -1.0;
  const auto r = line(line_goal(""));
  EXPECT_EQ(r.code, ResultCode::ABORTED);
  EXPECT_DOUBLE_EQ(r.result->fraction, -1.0);
  EXPECT_TRUE(contains(r.result->message, "no Cartesian path: TOTG failed"));
}

TEST_F(MoveToPoseNodeTest, LineInAnUnknownFrameIsRefused)
{
  start();
  const auto r = line(line_goal("nowhere"));
  EXPECT_EQ(r.code, ResultCode::ABORTED);
  EXPECT_TRUE(contains(r.result->message, "goal refused: no transform"));
  EXPECT_TRUE(std::isnan(r.result->fraction));
  EXPECT_TRUE(fake_->planned().empty());
}
