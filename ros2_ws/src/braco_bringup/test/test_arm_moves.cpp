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

// ArmMoves and the zero/park routines against FakeDriver: a node with arm_driver's topics and
// services that records every call and, when `follow` is set, jumps to each commanded pose.

#include <gtest/gtest.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "braco_bringup/arm_moves.hpp"
#include "braco_bringup/routines.hpp"
#include "mycobot.hpp"

using namespace std::chrono_literals;
using braco_bringup::Angles;
using braco_interfaces::msg::ArmCommand;
using braco_interfaces::srv::TakeControl;

namespace
{

constexpr double DEG_TO_RAD = M_PI / 180.0;

class FakeDriver : public rclcpp::Node
{
public:
  struct Options
  {
    bool publish_joints = true;
    bool follow = true;           // apply ALL_ANGLES commands to the joints
    bool grant_control = true;
    bool rest_service = true;
  };

  FakeDriver(const Angles & start, Options options)
  : rclcpp::Node("fake_arm_driver"), angles_(start), options_(options)
  {
    using std_srvs::srv::SetBool;
    using std_srvs::srv::Trigger;
    joints_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    command_sub_ = create_subscription<ArmCommand>(
      "arm/command", 10, [this](const ArmCommand & msg) {onCommand(msg);});
    power_srv_ = create_service<Trigger>(
      "arm/power_on", [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
        record("power_on");
        res->success = true;
      });
    release_srv_ = create_service<Trigger>(
      "arm/release_servos",
      [this](Trigger::Request::SharedPtr, Trigger::Response::SharedPtr res) {
        record("release_servos");
        res->success = true;
      });
    if (options_.rest_service) {
      rest_srv_ = create_service<SetBool>(
        "arm/set_read_only",
        [this](SetBool::Request::SharedPtr req, SetBool::Response::SharedPtr res) {
          record(req->data ? "rest_on" : "rest_off");
          res->success = true;
        });
    }
    control_srv_ = create_service<TakeControl>(
      "arm/take_control",
      [this](TakeControl::Request::SharedPtr req, TakeControl::Response::SharedPtr res) {
        std::lock_guard<std::mutex> lock(mutex_);
        control_requests_.push_back(*req);
        events_.push_back(req->release ? "release_control" : "take_control");
        res->success = req->release || options_.grant_control;
        res->owner = res->success && !req->release ? req->source : "";
        res->message = res->success ? "ok" : "refused";
      });
    if (options_.publish_joints) {
      timer_ = create_wall_timer(20ms, [this] {publishJoints();});
    }
  }

  std::vector<std::string> events()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return events_;
  }
  std::vector<ArmCommand> commands()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return commands_;
  }
  std::vector<TakeControl::Request> controlRequests()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return control_requests_;
  }

private:
  void record(const std::string & event)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    events_.push_back(event);
  }

  void onCommand(const ArmCommand & msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    commands_.push_back(msg);
    if (msg.type == ArmCommand::STOP) {
      events_.push_back("stop");
    } else if (msg.type == ArmCommand::ALL_ANGLES && options_.follow) {
      std::copy(msg.angles_deg.begin(), msg.angles_deg.end(), angles_.begin());
    }
  }

  void publishJoints()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    std::lock_guard<std::mutex> lock(mutex_);
    for (double a : angles_) {
      msg.position.push_back(a * DEG_TO_RAD);
    }
    joints_pub_->publish(msg);
  }

  std::mutex mutex_;
  Angles angles_;
  Options options_;
  std::vector<std::string> events_;
  std::vector<ArmCommand> commands_;
  std::vector<TakeControl::Request> control_requests_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joints_pub_;
  rclcpp::Subscription<ArmCommand>::SharedPtr command_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr release_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr rest_srv_;
  rclcpp::Service<TakeControl>::SharedPtr control_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class ArmMovesTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void TearDown() override
  {
    if (executor_) {executor_->cancel();}
    if (spinner_.joinable()) {spinner_.join();}
  }

  // Start the fake driver, spinning in its own thread, and the ArmMoves under test.
  void start(const Angles & from, FakeDriver::Options options = {})
  {
    driver_ = std::make_shared<FakeDriver>(from, options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(driver_);
    spinner_ = std::thread([this] {executor_->spin();});
    arm_ = std::make_shared<braco_bringup::ArmMoves>("arm_moves_test");
  }

  // Index of `event` in the driver's log, or -1.
  int indexOf(const std::string & event)
  {
    const auto events = driver_->events();
    const auto it = std::find(events.begin(), events.end(), event);
    return it == events.end() ? -1 : static_cast<int>(it - events.begin());
  }

  std::shared_ptr<FakeDriver> driver_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spinner_;
  std::shared_ptr<braco_bringup::ArmMoves> arm_;
};

}  // namespace

// ---- Driver and control -----------------------------------------------------------

TEST_F(ArmMovesTest, WaitForDriverFailsWithoutAngles)
{
  start(mycobot::ZERO_ANGLES, {false, true, true, true});
  EXPECT_FALSE(arm_->waitForDriver(1s));
}

TEST_F(ArmMovesTest, TakeControlIsForcedAsScript)
{
  start(mycobot::ZERO_ANGLES);
  ASSERT_TRUE(arm_->waitForDriver(5s));
  EXPECT_TRUE(arm_->takeControl());
  const auto requests = driver_->controlRequests();
  ASSERT_EQ(requests.size(), 1u);
  EXPECT_EQ(requests[0].source, "script");
  EXPECT_TRUE(requests[0].force);
  EXPECT_FALSE(requests[0].release);
}

TEST_F(ArmMovesTest, RefusedControlIsAFailure)
{
  start(mycobot::ZERO_ANGLES, {true, true, false, true});
  ASSERT_TRUE(arm_->waitForDriver(5s));
  EXPECT_FALSE(arm_->takeControl());
}

TEST_F(ArmMovesTest, ReleaseOnlyWhatWasTaken)
{
  start(mycobot::ZERO_ANGLES);
  ASSERT_TRUE(arm_->waitForDriver(5s));
  arm_->releaseControl();
  EXPECT_EQ(indexOf("release_control"), -1);
  ASSERT_TRUE(arm_->takeControl());
  arm_->releaseControl();
  arm_->releaseControl();   // the second one sends nothing
  const auto requests = driver_->controlRequests();
  ASSERT_EQ(requests.size(), 2u);
  EXPECT_TRUE(requests[1].release);
  EXPECT_EQ(requests[1].source, "script");
}

TEST_F(ArmMovesTest, MissingRestServiceIsNotFatal)
{
  start(mycobot::ZERO_ANGLES, {true, true, true, false});
  ASSERT_TRUE(arm_->waitForDriver(5s));
  EXPECT_FALSE(arm_->setRest(true));
}

// ---- moveTo ---------------------------------------------------------------------

TEST_F(ArmMovesTest, MoveToReachesThePoseInBoundedSteps)
{
  const Angles from = {60, -30, 0, 0, 0, 0};
  start(from);
  ASSERT_TRUE(arm_->waitForDriver(5s));
  EXPECT_TRUE(arm_->moveTo(mycobot::ZERO_ANGLES, 30, 2.0, 10s));

  const auto commands = driver_->commands();
  ASSERT_GE(commands.size(), 2u);   // 60 deg at most 15 ahead: several steps
  // First step: J1 (worst, 60 deg) leads by 15, J2 by the same fraction.
  EXPECT_NEAR(commands[0].angles_deg[0], 45.0, 0.05);
  EXPECT_NEAR(commands[0].angles_deg[1], -22.5, 0.05);
  for (const auto & c : commands) {
    EXPECT_EQ(c.type, ArmCommand::ALL_ANGLES);
    EXPECT_EQ(c.source, "script");
    EXPECT_EQ(c.speed, 30);
  }
  EXPECT_EQ(indexOf("stop"), -1);
}

TEST_F(ArmMovesTest, MoveToAlreadyThereSendsNothing)
{
  start(mycobot::ZERO_ANGLES);
  ASSERT_TRUE(arm_->waitForDriver(5s));
  EXPECT_TRUE(arm_->moveTo(mycobot::ZERO_ANGLES, 30, 2.0, 5s));
  EXPECT_TRUE(driver_->commands().empty());
}

TEST_F(ArmMovesTest, MoveToTimeoutStopsTheArm)
{
  start({60, 0, 0, 0, 0, 0}, {true, false, true, true});   // the arm never moves
  ASSERT_TRUE(arm_->waitForDriver(5s));
  EXPECT_FALSE(arm_->moveTo(mycobot::ZERO_ANGLES, 30, 2.0, 1s));
  arm_->wait(200ms);
  EXPECT_NE(indexOf("stop"), -1);
  // Resent while waiting, so the driver's 300 ms watchdog never fires on a live program.
  EXPECT_GE(driver_->commands().size(), 5u);
}

TEST_F(ArmMovesTest, MoveToWithoutFeedbackStopsTheArm)
{
  start(mycobot::ZERO_ANGLES, {false, true, true, true});
  EXPECT_FALSE(arm_->moveTo(mycobot::PARK_ANGLES, 30, 2.0, 1s));
  arm_->wait(200ms);
  EXPECT_NE(indexOf("stop"), -1);
}

// ---- Routines -------------------------------------------------------------------

TEST_F(ArmMovesTest, ZeroLeavesRestThenPowersOnThenMoves)
{
  start({20, 10, -10, 0, 0, 0});
  EXPECT_TRUE(braco_bringup::zero(*arm_));
  EXPECT_EQ(
    driver_->events(),
    (std::vector<std::string>{"take_control", "rest_off", "power_on"}));
  EXPECT_FALSE(driver_->commands().empty());
}

TEST_F(ArmMovesTest, ParkMovesThenReleasesThenRests)
{
  start(mycobot::ZERO_ANGLES);
  EXPECT_TRUE(braco_bringup::park(*arm_));
  EXPECT_EQ(
    driver_->events(),
    (std::vector<std::string>{"take_control", "rest_off", "release_servos", "rest_on"}));
  const auto last = driver_->commands().back();
  for (size_t i = 0; i < 6; ++i) {
    EXPECT_NEAR(last.angles_deg[i], mycobot::PARK_ANGLES[i], mycobot::POSE_TOLERANCE_DEG);
  }
}

TEST_F(ArmMovesTest, RoutinesStopWhenControlIsRefused)
{
  start(mycobot::ZERO_ANGLES, {true, true, false, true});
  EXPECT_FALSE(braco_bringup::park(*arm_));
  EXPECT_FALSE(braco_bringup::zero(*arm_));
  EXPECT_EQ(
    driver_->events(),
    (std::vector<std::string>{"take_control", "take_control"}));
  EXPECT_TRUE(driver_->commands().empty());
}
