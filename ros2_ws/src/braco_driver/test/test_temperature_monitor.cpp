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

// TemperatureMonitor with no LCD (lcd_bus is a missing path) and a test node in place of
// arm_driver: it publishes /arm/servo_temperatures, records /diagnostics and, when offered,
// answers /arm/set_read_only. The park program is `true` or `false`, so it runs at once.

#include <gtest/gtest.h>

#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "braco_driver/temperature_monitor.hpp"

using namespace std::chrono_literals;
using braco_interfaces::msg::ServoTemperatures;
using diagnostic_msgs::msg::DiagnosticArray;
using diagnostic_msgs::msg::DiagnosticStatus;
using std_srvs::srv::SetBool;

namespace
{

// Poll `done` until it holds or `timeout` expires.
bool waitUntil(const std::function<bool()> & done, std::chrono::milliseconds timeout = 3s)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    if (done()) {return true;}
    std::this_thread::sleep_for(10ms);
  }
  return done();
}

class TemperatureMonitorTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void TearDown() override
  {
    if (executor_) {executor_->cancel();}
    if (spinner_.joinable()) {spinner_.join();}
  }

  // The monitor (no LCD, park off unless `extra` says otherwise) and the test node.
  void start(const std::vector<rclcpp::Parameter> & extra = {})
  {
    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("lcd_bus", "/dev/does-not-exist"),
      rclcpp::Parameter("park_on_hot", false),
      rclcpp::Parameter("park_command", std::vector<std::string>{"true"})};
    params.insert(params.end(), extra.begin(), extra.end());
    monitor_ = std::make_shared<braco_driver::TemperatureMonitor>(
      rclcpp::NodeOptions().parameter_overrides(params));

    driver_ = std::make_shared<rclcpp::Node>("temperature_monitor_test_driver");
    temp_pub_ = driver_->create_publisher<ServoTemperatures>("arm/servo_temperatures", 10);
    diag_sub_ = driver_->create_subscription<DiagnosticArray>(
      "diagnostics", 10, [this](const DiagnosticArray & msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        diags_.push_back(msg);
      });

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(monitor_);
    executor_->add_node(driver_);
    spinner_ = std::thread([this] {executor_->spin();});
    ASSERT_TRUE(waitUntil([this] {return temp_pub_->get_subscription_count() > 0;}));
    ASSERT_TRUE(waitUntil([this] {return diag_sub_->get_publisher_count() > 0;}));
  }

  // arm_driver's /arm/set_read_only, answering `success`.
  void offerRest(bool success = true)
  {
    rest_srv_ = driver_->create_service<SetBool>(
      "arm/set_read_only",
      [this, success](SetBool::Request::ConstSharedPtr req, SetBool::Response::SharedPtr res) {
        std::lock_guard<std::mutex> lock(mutex_);
        rest_calls_.push_back(req->data);
        res->success = success;
        res->message = success ? "rest mode on" : "refused by the test";
      });
    // Once a client of the same graph sees it, so does the monitor's
    auto probe = driver_->create_client<SetBool>("arm/set_read_only");
    ASSERT_TRUE(probe->wait_for_service(5s));
  }

  // Publish one reading and return the /diagnostics it produced.
  DiagnosticArray send(const std::vector<int> & celsius, bool valid = true)
  {
    const size_t before = diagCount();
    ServoTemperatures msg;
    msg.header.stamp = driver_->now();
    msg.valid = valid;
    for (size_t j = 0; j < celsius.size() && j < msg.celsius.size(); ++j) {
      msg.celsius[j] = static_cast<int16_t>(celsius[j]);
    }
    temp_pub_->publish(msg);
    if (!waitUntil([&] {return diagCount() > before;})) {return DiagnosticArray();}
    std::lock_guard<std::mutex> lock(mutex_);
    return diags_.back();
  }

  size_t diagCount()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return diags_.size();
  }

  size_t restCalls()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return rest_calls_.size();
  }

  // The STALE status published after `from` diagnostics, waiting up to `timeout`.
  bool staleSince(size_t from, const std::string & why, std::chrono::milliseconds timeout = 3s)
  {
    return waitUntil(
      [&] {
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = from; i < diags_.size(); ++i) {
          const auto & s = diags_[i].status;
          if (s.size() == 1 && s[0].level == DiagnosticStatus::STALE && s[0].message == why) {
            return true;
          }
        }
        return false;
      }, timeout);
  }

  std::shared_ptr<braco_driver::TemperatureMonitor> monitor_;
  std::shared_ptr<rclcpp::Node> driver_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spinner_;

  std::mutex mutex_;
  std::vector<DiagnosticArray> diags_;
  std::vector<bool> rest_calls_;

  rclcpp::Publisher<ServoTemperatures>::SharedPtr temp_pub_;
  rclcpp::Subscription<DiagnosticArray>::SharedPtr diag_sub_;
  rclcpp::Service<SetBool>::SharedPtr rest_srv_;
};

}  // namespace

// ---- Diagnostics ------------------------------------------------------------------

TEST_F(TemperatureMonitorTest, OneStatusPerJointAtEachLevel)
{
  start();
  offerRest();

  auto diag = send({35, 36, 37, 38, 39, 40});
  ASSERT_EQ(diag.status.size(), 6u);
  EXPECT_EQ(diag.status[0].name, "mycobot: J1 servo temperature");
  EXPECT_EQ(diag.status[0].hardware_id, "mycobot280");
  EXPECT_EQ(diag.status[0].level, DiagnosticStatus::OK);
  EXPECT_EQ(diag.status[0].message, "35 C (normal)");
  ASSERT_EQ(diag.status[5].values.size(), 1u);
  EXPECT_EQ(diag.status[5].values[0].key, "celsius");
  EXPECT_EQ(diag.status[5].values[0].value, "40");

  diag = send({35, 58, 35, 35, 35, 35});
  ASSERT_EQ(diag.status.size(), 6u);
  EXPECT_EQ(diag.status[1].level, DiagnosticStatus::WARN);
  EXPECT_EQ(diag.status[1].message, "58 C (warm)");

  diag = send({35, 58, 35, 70, 35, 35});
  ASSERT_EQ(diag.status.size(), 6u);
  EXPECT_EQ(diag.status[1].level, DiagnosticStatus::WARN);
  EXPECT_EQ(diag.status[3].level, DiagnosticStatus::ERROR);
  EXPECT_EQ(diag.status[3].message, "70 C (HOT)");

  diag = send({35, 35, 35, 35, 35, 35});   // back to normal
  EXPECT_EQ(diag.status[3].level, DiagnosticStatus::OK);
}

TEST_F(TemperatureMonitorTest, InvalidReadingIsStale)
{
  start();
  send({35, 35, 35, 35, 35, 35});
  const size_t from = diagCount();
  const auto diag = send({}, false);
  ASSERT_EQ(diag.status.size(), 1u);
  EXPECT_EQ(diag.status[0].name, "mycobot: servo temperatures");
  EXPECT_TRUE(staleSince(from, "the arm did not answer the temperature read"));
}

TEST_F(TemperatureMonitorTest, NoReadingsForStaleTimeoutIsStale)
{
  start({rclcpp::Parameter("stale_timeout_s", 0.3)});
  const std::string why = "no temperatures from arm_driver - is it running?";
  EXPECT_TRUE(staleSince(0, why));
  const size_t from = diagCount();
  EXPECT_TRUE(staleSince(from, why));   // repeated every second while nothing comes
}

// ---- Rest on overheat -------------------------------------------------------------

TEST_F(TemperatureMonitorTest, HotWithoutAutoRestIsOnlyReported)
{
  start({rclcpp::Parameter("auto_rest_on_hot", false)});
  offerRest();
  send({35, 35, 35, 70, 35, 35});
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(restCalls(), 0u);
}

TEST_F(TemperatureMonitorTest, HotRequestsRestModeOncePerEpisode)
{
  start();   // park_on_hot false: rest mode only
  offerRest();
  send({35, 35, 35, 70, 35, 35});
  ASSERT_TRUE(waitUntil([this] {return restCalls() == 1;}));
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_TRUE(rest_calls_[0]);
  }
  send({35, 35, 35, 66, 35, 35});   // the same episode
  send({35, 35, 35, 62, 35, 35});   // above REARM_BELOW_C: still the same episode
  send({35, 35, 35, 70, 35, 35});
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(restCalls(), 1u);

  send({35, 35, 35, 50, 35, 35});   // cooled down: a new episode may start
  send({35, 35, 35, 70, 35, 35});
  EXPECT_TRUE(waitUntil([this] {return restCalls() == 2;}));
}

TEST_F(TemperatureMonitorTest, UnreachableDriverIsAskedAgainOnTheNextHotReading)
{
  start();
  send({35, 35, 35, 70, 35, 35});   // no service yet
  offerRest();
  EXPECT_EQ(restCalls(), 0u);
  send({35, 35, 35, 70, 35, 35});
  EXPECT_TRUE(waitUntil([this] {return restCalls() == 1;}));
}

TEST_F(TemperatureMonitorTest, RefusedRestModeIsAskedAgain)
{
  start();
  offerRest(false);
  send({35, 35, 35, 70, 35, 35});
  ASSERT_TRUE(waitUntil([this] {return restCalls() == 1;}));
  // Hot readings until the refused answer has re-armed the episode: the ones before it are
  // still the first episode and ask nothing; without the re-arm, none ever asks again.
  // Every answer is refused and re-arms, so >= 2.
  EXPECT_TRUE(
    waitUntil(
      [this] {
        send({35, 35, 35, 70, 35, 35});
        return restCalls() >= 2;
      }));
}

TEST_F(TemperatureMonitorTest, ParkRunsThenRestModeIsConfirmed)
{
  start({rclcpp::Parameter("park_on_hot", true)});
  offerRest();
  send({35, 35, 35, 70, 35, 35});
  send({35, 35, 35, 70, 35, 35});   // park already running or done: not started again
  // `true` ends at once; the 1 s timer sees it and asks for rest mode
  EXPECT_TRUE(waitUntil([this] {return restCalls() == 1;}));
  std::this_thread::sleep_for(1500ms);
  EXPECT_EQ(restCalls(), 1u);
}

TEST_F(TemperatureMonitorTest, FailedParkStillPutsTheArmToRest)
{
  start(
    {rclcpp::Parameter("park_on_hot", true),
      rclcpp::Parameter("park_command", std::vector<std::string>{"false"})});
  offerRest();
  send({35, 35, 35, 70, 35, 35});
  EXPECT_TRUE(waitUntil([this] {return restCalls() == 1;}));
}
