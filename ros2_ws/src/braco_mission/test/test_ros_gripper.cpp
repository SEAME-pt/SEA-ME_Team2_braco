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

// ActionGripper and ServiceGripper against a fake gripper_action_controller and a fake
// /arm/set_gripper, served by a test node in the same executor (#148).

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "braco_mission/ros_gripper.hpp"

using namespace std::chrono_literals;
using braco_mission::ActionGripper;
using braco_mission::GripperConfig;
using braco_mission::ServiceGripper;
using braco_mission::make_gripper;
using Command = ActionGripper::Command;
using CommandHandle = rclcpp_action::ServerGoalHandle<Command>;
using SetBool = ServiceGripper::SetBool;

namespace
{

// How the fake gripper_action_controller ends a goal
enum class Answer {Succeed, Abort, Reject, Hang};

bool contains(const std::string & text, const std::string & part)
{
  return text.find(part) != std::string::npos;
}

class RosGripperTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    node_ = std::make_shared<rclcpp::Node>("gripper_under_test");
    helper_ = std::make_shared<rclcpp::Node>("gripper_test");
    config_.timeout_s = 0.5;
    config_.settle_s = 0.0;
  }

  void TearDown() override
  {
    if (executor_) {executor_->cancel();}
    if (spinner_.joinable()) {spinner_.join();}
  }

  void spin()
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_->add_node(helper_);
    spinner_ = std::thread([this] {executor_->spin();});
  }

  // The fake gripper_action_controller: records the commanded positions, answers `answer_`
  void serve_action()
  {
    server_ = rclcpp_action::create_server<Command>(
      helper_, ActionGripper::ACTION,
      [this](const rclcpp_action::GoalUUID &, std::shared_ptr<const Command::Goal> goal) {
        std::lock_guard<std::mutex> lock(mutex_);
        names_ = goal->command.name;
        positions_.push_back(goal->command.position.at(0));
        if (answer_ == Answer::Reject) {return rclcpp_action::GoalResponse::REJECT;}
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
      },
      [this](std::shared_ptr<CommandHandle>) {
        ++cancels_;
        return rclcpp_action::CancelResponse::ACCEPT;
      },
      [this](std::shared_ptr<CommandHandle> goal) {
        auto result = std::make_shared<Command::Result>();
        if (answer_ == Answer::Succeed) {
          result->reached_goal = true;
          goal->succeed(result);
        } else if (answer_ == Answer::Abort) {
          goal->abort(result);
        } else {
          hanging_ = goal;   // never ends on its own
        }
      });
  }

  // The fake /arm/set_gripper: records the requests, answers `granted_`
  void serve_service()
  {
    service_ = helper_->create_service<SetBool>(
      ServiceGripper::SERVICE,
      [this](const SetBool::Request::SharedPtr request, SetBool::Response::SharedPtr response) {
        std::lock_guard<std::mutex> lock(mutex_);
        requests_.push_back(request->data);
        response->success = granted_;
        response->message = granted_ ? "" : "set gripper refused: STOP latched";
      });
  }

  std::vector<double> positions()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return positions_;
  }

  std::vector<bool> requests()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return requests_;
  }

  rclcpp::Node::SharedPtr node_;
  rclcpp::Node::SharedPtr helper_;
  GripperConfig config_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spinner_;

  std::atomic<Answer> answer_{Answer::Succeed};
  std::atomic<bool> granted_{true};
  std::atomic<int> cancels_{0};
  rclcpp_action::Server<Command>::SharedPtr server_;
  std::shared_ptr<CommandHandle> hanging_;
  rclcpp::Service<SetBool>::SharedPtr service_;
  std::mutex mutex_;
  std::vector<std::string> names_;
  std::vector<double> positions_;
  std::vector<bool> requests_;
};

}  // namespace

// ---- factory ---------------------------------------------------------------------------

TEST_F(RosGripperTest, FactoryBuildsTheNamedGripper)
{
  config_.kind = "sim";
  EXPECT_NE(dynamic_cast<ActionGripper *>(make_gripper(*node_, config_).get()), nullptr);
  config_.kind = "real";
  EXPECT_NE(dynamic_cast<ServiceGripper *>(make_gripper(*node_, config_).get()), nullptr);
}

TEST_F(RosGripperTest, FactoryRefusesAnUnknownKind)
{
  config_.kind = "magnet";
  EXPECT_THROW(make_gripper(*node_, config_), std::invalid_argument);
}

// ---- ActionGripper (Gazebo) ------------------------------------------------------------

TEST_F(RosGripperTest, ActionGripperCommandsTheMasterJoint)
{
  config_.open_position = 0.15;
  config_.closed_position = -0.74;
  ActionGripper gripper(*node_, config_);
  serve_action();
  spin();
  std::string message;
  EXPECT_TRUE(gripper.open(message)) << message;
  EXPECT_TRUE(gripper.close(message)) << message;
  EXPECT_EQ(positions(), (std::vector<double>{0.15, -0.74}));
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_EQ(names_, std::vector<std::string>{"gripper_controller"});
}

TEST_F(RosGripperTest, ActionGripperAbortedIsAFailure)
{
  ActionGripper gripper(*node_, config_);
  answer_ = Answer::Abort;
  serve_action();
  spin();
  std::string message;
  EXPECT_FALSE(gripper.close(message));
  EXPECT_TRUE(contains(message, "close gripper: aborted")) << message;
}

TEST_F(RosGripperTest, ActionGripperRejectedIsAFailure)
{
  ActionGripper gripper(*node_, config_);
  answer_ = Answer::Reject;
  serve_action();
  spin();
  std::string message;
  EXPECT_FALSE(gripper.open(message));
  EXPECT_TRUE(contains(message, "refused")) << message;
}

TEST_F(RosGripperTest, ActionGripperTimesOutAndCancels)
{
  ActionGripper gripper(*node_, config_);
  answer_ = Answer::Hang;
  serve_action();
  spin();
  std::string message;
  EXPECT_FALSE(gripper.close(message));
  EXPECT_TRUE(contains(message, "not done in")) << message;
  for (int i = 0; i < 50 && cancels_ == 0; ++i) {
    std::this_thread::sleep_for(20ms);
  }
  EXPECT_EQ(cancels_, 1);
}

TEST_F(RosGripperTest, ActionGripperWithoutControllerFails)
{
  ActionGripper gripper(*node_, config_);
  spin();
  std::string message;
  EXPECT_FALSE(gripper.open(message));
  EXPECT_TRUE(contains(message, "is not available")) << message;
}

// ---- ServiceGripper (real arm) ---------------------------------------------------------

TEST_F(RosGripperTest, ServiceGripperClosesWithTrue)
{
  ServiceGripper gripper(*node_, config_);
  serve_service();
  spin();
  std::string message;
  EXPECT_TRUE(gripper.open(message)) << message;
  EXPECT_TRUE(gripper.close(message)) << message;
  EXPECT_EQ(requests(), (std::vector<bool>{false, true}));
}

TEST_F(RosGripperTest, ServiceGripperWaitsForTheGripperToMove)
{
  config_.settle_s = 0.3;
  ServiceGripper gripper(*node_, config_);
  serve_service();
  spin();
  std::string message;
  const auto start = std::chrono::steady_clock::now();
  EXPECT_TRUE(gripper.close(message)) << message;
  EXPECT_GE(std::chrono::steady_clock::now() - start, 300ms);
}

TEST_F(RosGripperTest, ServiceGripperRefusalSaysWhy)
{
  ServiceGripper gripper(*node_, config_);
  granted_ = false;
  serve_service();
  spin();
  std::string message;
  EXPECT_FALSE(gripper.close(message));
  EXPECT_EQ(message, "set gripper refused: STOP latched");
}

TEST_F(RosGripperTest, ServiceGripperWithoutDriverFails)
{
  ServiceGripper gripper(*node_, config_);
  spin();
  std::string message;
  EXPECT_FALSE(gripper.close(message));
  EXPECT_TRUE(contains(message, "is not available")) << message;
}
