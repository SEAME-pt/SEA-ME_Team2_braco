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

// ArmDriver against PtyArm (mycobot_cpp/test/pty_arm.hpp): the node opens the pseudo-terminal as
// its port, a client node in the same executor uses its topics and services, and the frames the
// fake arm received show what reached the serial port.

#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "braco_driver/arm_driver.hpp"
#include "pty_arm.hpp"

using namespace std::chrono_literals;
using braco_interfaces::msg::ArmCommand;
using braco_interfaces::msg::JointTarget;
using braco_interfaces::msg::ServoTemperatures;
using braco_interfaces::msg::ServoVoltages;
using braco_interfaces::srv::TakeControl;
using mycobot::fake::PtyArm;
using std_srvs::srv::SetBool;
using std_srvs::srv::Trigger;
namespace protocol = mycobot::protocol;

namespace
{

constexpr double DEG_TO_RAD = M_PI / 180.0;

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

class ArmDriverTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void SetUp() override
  {
    arm_ = std::make_unique<PtyArm>();
    arm_->serve();
  }

  void TearDown() override
  {
    if (executor_) {executor_->cancel();}
    if (spinner_.joinable()) {spinner_.join();}
    client_.reset();
    driver_.reset();   // its destructor sends the final stop, so before the fake arm goes
    arm_.reset();
  }

  // The driver on the fake arm's port (temperatures every 0.2 s), and the client node, both
  // spinning in one thread.
  void start(const std::vector<rclcpp::Parameter> & extra = {})
  {
    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("port", arm_->path()),
      rclcpp::Parameter("temp_period_s", 0.2)};
    params.insert(params.end(), extra.begin(), extra.end());
    driver_ = std::make_shared<braco_driver::ArmDriver>(
      rclcpp::NodeOptions().parameter_overrides(params));

    client_ = std::make_shared<rclcpp::Node>("arm_driver_test_client");
    const auto latched = rclcpp::QoS(1).transient_local();
    command_pub_ = client_->create_publisher<ArmCommand>("arm/command", 10);
    led_pub_ = client_->create_publisher<std_msgs::msg::ColorRGBA>("arm/led", 10);
    joint_sub_ = client_->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10, [this](const sensor_msgs::msg::JointState & msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        joints_.push_back(msg);
      });
    temp_sub_ = client_->create_subscription<ServoTemperatures>(
      "arm/servo_temperatures", 10, [this](const ServoTemperatures & msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        temps_.push_back(msg);
      });
    volt_sub_ = client_->create_subscription<ServoVoltages>(
      "arm/servo_voltages", 10, [this](const ServoVoltages & msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        volts_.push_back(msg);
      });
    owner_sub_ = client_->create_subscription<std_msgs::msg::String>(
      "arm/control_owner", latched, [this](const std_msgs::msg::String & msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        owners_.push_back(msg.data);
      });
    read_only_sub_ = client_->create_subscription<std_msgs::msg::Bool>(
      "arm/read_only", latched, [this](const std_msgs::msg::Bool & msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        read_only_.push_back(msg.data);
      });
    take_client_ = client_->create_client<TakeControl>("arm/take_control");
    power_client_ = client_->create_client<Trigger>("arm/power_on");
    release_client_ = client_->create_client<Trigger>("arm/release_servos");
    gripper_client_ = client_->create_client<SetBool>("arm/set_gripper");
    rest_client_ = client_->create_client<SetBool>("arm/set_read_only");

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(driver_);
    executor_->add_node(client_);
    spinner_ = std::thread([this] {executor_->spin();});
    // A message published before the subscription is matched would be lost
    ASSERT_TRUE(waitUntil([this] {return command_pub_->get_subscription_count() > 0;}));
    ASSERT_TRUE(waitUntil([this] {return led_pub_->get_subscription_count() > 0;}));
  }

  template<typename Srv>
  typename Srv::Response::SharedPtr call(
    const typename rclcpp::Client<Srv>::SharedPtr & client,
    typename Srv::Request::SharedPtr request)
  {
    if (!client->wait_for_service(5s)) {return nullptr;}
    auto future = client->async_send_request(request);
    if (future.wait_for(5s) != std::future_status::ready) {return nullptr;}
    return future.get();
  }

  TakeControl::Response::SharedPtr take(
    const std::string & source, bool force = false, bool release = false)
  {
    auto req = std::make_shared<TakeControl::Request>();
    req->source = source;
    req->force = force;
    req->release = release;
    return call<TakeControl>(take_client_, req);
  }

  Trigger::Response::SharedPtr trigger(const rclcpp::Client<Trigger>::SharedPtr & client)
  {
    return call<Trigger>(client, std::make_shared<Trigger::Request>());
  }

  SetBool::Response::SharedPtr setBool(const rclcpp::Client<SetBool>::SharedPtr & client, bool on)
  {
    auto req = std::make_shared<SetBool::Request>();
    req->data = on;
    return call<SetBool>(client, req);
  }

  void sendJoints(const std::string & source, const std::vector<JointTarget> & targets)
  {
    ArmCommand msg;
    msg.type = ArmCommand::JOINTS;
    msg.targets = targets;
    msg.source = source;
    command_pub_->publish(msg);
  }

  void sendType(const std::string & source, uint8_t type)
  {
    ArmCommand msg;
    msg.type = type;
    msg.source = source;
    command_pub_->publish(msg);
  }

  static JointTarget target(uint8_t joint, double angle_deg, uint8_t speed = 30)
  {
    JointTarget t;
    t.joint = joint;
    t.angle_deg = angle_deg;
    t.speed = speed;
    return t;
  }

  size_t frames(uint8_t cmd) const {return arm_->received(cmd).size();}

  std::string lastOwner()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return owners_.empty() ? "?" : owners_.back();
  }

  std::unique_ptr<PtyArm> arm_;
  std::shared_ptr<braco_driver::ArmDriver> driver_;
  std::shared_ptr<rclcpp::Node> client_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spinner_;

  std::mutex mutex_;
  std::vector<sensor_msgs::msg::JointState> joints_;
  std::vector<ServoTemperatures> temps_;
  std::vector<ServoVoltages> volts_;
  std::vector<std::string> owners_;
  std::vector<bool> read_only_;

  rclcpp::Publisher<ArmCommand>::SharedPtr command_pub_;
  rclcpp::Publisher<std_msgs::msg::ColorRGBA>::SharedPtr led_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<ServoTemperatures>::SharedPtr temp_sub_;
  rclcpp::Subscription<ServoVoltages>::SharedPtr volt_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr owner_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr read_only_sub_;
  rclcpp::Client<TakeControl>::SharedPtr take_client_;
  rclcpp::Client<Trigger>::SharedPtr power_client_;
  rclcpp::Client<Trigger>::SharedPtr release_client_;
  rclcpp::Client<SetBool>::SharedPtr gripper_client_;
  rclcpp::Client<SetBool>::SharedPtr rest_client_;
};

}  // namespace

// ---- Start and feedback -----------------------------------------------------------

TEST_F(ArmDriverTest, MissingPortThrows)
{
  const auto options = rclcpp::NodeOptions().parameter_overrides(
    {rclcpp::Parameter("port", "/dev/does-not-exist")});
  EXPECT_THROW(braco_driver::ArmDriver{options}, std::exception);
}

TEST_F(ArmDriverTest, PublishesAnglesTemperaturesAndVoltages)
{
  arm_->update(
    [](auto & s) {
      s.angles = {10, -20, 30, -40, 50, -60};
      s.temps = {31, 32, 33, 34, 35, 36};
      s.decivolts = {120, 121, 122, 123, 124, 125};
    });
  start();
  ASSERT_TRUE(
    waitUntil(
      [this] {
        std::lock_guard<std::mutex> lock(mutex_);
        return !joints_.empty() && temps_.size() >= 2 && !volts_.empty();
      }));
  std::lock_guard<std::mutex> lock(mutex_);
  const auto & j = joints_.back();
  ASSERT_EQ(j.name.size(), 6u);
  EXPECT_EQ(j.name[0], "joint1");
  EXPECT_EQ(j.name[5], "joint6");
  ASSERT_EQ(j.position.size(), 6u);
  EXPECT_NEAR(j.position[0], 10 * DEG_TO_RAD, 1e-6);
  EXPECT_NEAR(j.position[5], -60 * DEG_TO_RAD, 1e-6);
  EXPECT_TRUE(temps_.back().valid);
  EXPECT_EQ(temps_.back().celsius[0], 31);
  EXPECT_EQ(temps_.back().celsius[5], 36);
  EXPECT_TRUE(volts_.back().valid);
  EXPECT_NEAR(volts_.back().volts[0], 12.0, 1e-3);
  EXPECT_NEAR(volts_.back().volts[5], 12.5, 1e-3);
}

TEST_F(ArmDriverTest, SilentArmPublishesInvalidReadingsAndNoAngles)
{
  arm_->update([](auto & s) {s.silent = true;});
  start();
  ASSERT_TRUE(
    waitUntil(
      [this] {
        std::lock_guard<std::mutex> lock(mutex_);
        return !temps_.empty() && !volts_.empty();
      }, 10s));
  std::lock_guard<std::mutex> lock(mutex_);
  EXPECT_FALSE(temps_.back().valid);
  EXPECT_FALSE(volts_.back().valid);
  EXPECT_TRUE(joints_.empty());
}

// ---- Control owner ----------------------------------------------------------------

TEST_F(ArmDriverTest, TakeControlRules)
{
  start();
  ASSERT_TRUE(waitUntil([this] {return lastOwner() == "";}));

  auto r = take("web");
  ASSERT_TRUE(r);
  EXPECT_TRUE(r->success);
  EXPECT_EQ(r->owner, "web");
  EXPECT_EQ(r->message, "control taken");
  EXPECT_TRUE(waitUntil([this] {return lastOwner() == "web";}));

  r = take("web");   // already ours
  EXPECT_TRUE(r->success);

  r = take("joystick");
  EXPECT_FALSE(r->success);
  EXPECT_EQ(r->owner, "web");
  EXPECT_EQ(r->message, "the arm is controlled by web");

  r = take("");
  EXPECT_FALSE(r->success);
  EXPECT_EQ(r->message, "source must not be empty");

  r = take("joystick", false, true);
  EXPECT_FALSE(r->success);
  EXPECT_EQ(r->message, "only the owner (web) can release the control");

  const size_t stops = frames(protocol::STOP);
  r = take("joystick", true);
  EXPECT_TRUE(r->success);
  EXPECT_EQ(r->owner, "joystick");
  EXPECT_EQ(r->message, "control taken from web");
  EXPECT_TRUE(waitUntil([this] {return lastOwner() == "joystick";}));
  EXPECT_TRUE(waitUntil([&] {return frames(protocol::STOP) > stops;}));   // the arm stopped

  r = take("joystick", false, true);
  EXPECT_TRUE(r->success);
  EXPECT_EQ(r->owner, "");
  EXPECT_EQ(r->message, "control released");
  EXPECT_TRUE(waitUntil([this] {return lastOwner() == "";}));

  r = take("web", false, true);   // releasing a free control is fine
  EXPECT_TRUE(r->success);
}

TEST_F(ArmDriverTest, IdleControlIsFreed)
{
  start({rclcpp::Parameter("control_idle_s", 0.3)});
  ASSERT_TRUE(take("web")->success);
  ASSERT_TRUE(waitUntil([this] {return lastOwner() == "web";}));
  EXPECT_TRUE(waitUntil([this] {return lastOwner() == "";}));
}

// ---- Motion -----------------------------------------------------------------------

TEST_F(ArmDriverTest, MotionOnlyFromTheOwner)
{
  start({rclcpp::Parameter("command_timeout_ms", 5000)});
  ASSERT_TRUE(take("web")->success);

  sendJoints("joystick", {target(1, 12.5)});
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(frames(protocol::SEND_ANGLE), 0u);

  sendJoints("web", {target(7, 10), target(2, 12.5, 40), target(3, -5)});   // 7 is not a joint
  ASSERT_TRUE(waitUntil([this] {return frames(protocol::SEND_ANGLE) >= 2;}));
  std::this_thread::sleep_for(200ms);
  const auto sent = arm_->received(protocol::SEND_ANGLE);
  ASSERT_EQ(sent.size(), 2u);
  EXPECT_EQ(sent[0].data[0], 2);
  EXPECT_EQ(mycobot::fake::readInt16(sent[0].data, 1), 1250);
  EXPECT_EQ(sent[0].data[3], 40);
  EXPECT_EQ(sent[1].data[0], 3);
  EXPECT_EQ(mycobot::fake::readInt16(sent[1].data, 1), -500);

  ArmCommand all;
  all.type = ArmCommand::ALL_ANGLES;
  all.angles_deg = {20, 0, 0, 0, 0, 0};
  all.speed = 25;
  all.source = "web";
  command_pub_->publish(all);
  ASSERT_TRUE(waitUntil([this] {return frames(protocol::SEND_ANGLES) == 1;}));
  // The fake arm moved there, and /joint_states follows
  EXPECT_TRUE(
    waitUntil(
      [this] {
        std::lock_guard<std::mutex> lock(mutex_);
        return !joints_.empty() && std::abs(joints_.back().position[0] - 20 * DEG_TO_RAD) < 1e-6;
      }));

  sendType("web", 9);   // unknown type: nothing is sent
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(frames(protocol::SEND_ANGLE), 2u);
  EXPECT_EQ(frames(protocol::SEND_ANGLES), 1u);

  const size_t stops = frames(protocol::STOP);
  sendType("joystick", ArmCommand::STOP);   // STOP passes, whoever sends it
  EXPECT_TRUE(waitUntil([&] {return frames(protocol::STOP) > stops;}));
}

TEST_F(ArmDriverTest, WatchdogStopsOnceWhenCommandsStop)
{
  start({rclcpp::Parameter("command_timeout_ms", 100)});
  ASSERT_TRUE(take("web")->success);
  sendJoints("web", {target(1, 5)});
  ASSERT_TRUE(waitUntil([this] {return frames(protocol::SEND_ANGLE) == 1;}));
  EXPECT_TRUE(waitUntil([this] {return frames(protocol::STOP) == 1;}));
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(frames(protocol::STOP), 1u);   // once, not on every pass
}

// ---- Rest mode and services -------------------------------------------------------

TEST_F(ArmDriverTest, RestModeRefusesMotionUntilLeft)
{
  start({rclcpp::Parameter("read_only", true)});
  EXPECT_TRUE(
    waitUntil(
      [this] {
        std::lock_guard<std::mutex> lock(mutex_);
        return !read_only_.empty() && read_only_.back();
      }));
  ASSERT_TRUE(take("web")->success);

  auto r = trigger(power_client_);
  ASSERT_TRUE(r);
  EXPECT_FALSE(r->success);
  EXPECT_EQ(r->message, "power on refused: rest mode (call arm/set_read_only false)");
  auto g = setBool(gripper_client_, true);
  ASSERT_TRUE(g);
  EXPECT_FALSE(g->success);
  EXPECT_EQ(g->message, "set gripper refused: rest mode (call arm/set_read_only false)");
  sendJoints("web", {target(1, 5)});
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(frames(protocol::SEND_ANGLE), 0u);
  EXPECT_EQ(frames(protocol::POWER_ON), 0u);
  EXPECT_EQ(frames(protocol::SET_GRIPPER_STATE), 0u);
  {
    std::lock_guard<std::mutex> lock(mutex_);
    EXPECT_FALSE(joints_.empty());   // still monitoring
  }

  g = setBool(rest_client_, false);
  ASSERT_TRUE(g);
  EXPECT_TRUE(g->success);
  EXPECT_EQ(g->message, "rest mode off");
  EXPECT_TRUE(
    waitUntil(
      [this] {
        std::lock_guard<std::mutex> lock(mutex_);
        return !read_only_.back();
      }));
  r = trigger(power_client_);
  EXPECT_TRUE(r->success);
  EXPECT_EQ(r->message, "power on");
  EXPECT_TRUE(waitUntil([this] {return frames(protocol::POWER_ON) == 1;}));

  g = setBool(rest_client_, true);
  EXPECT_EQ(g->message, "rest mode on");
}

TEST_F(ArmDriverTest, GripperReleaseAndLed)
{
  start();
  auto g = setBool(gripper_client_, true);
  ASSERT_TRUE(g);
  EXPECT_TRUE(g->success);
  EXPECT_EQ(g->message, "close gripper");
  g = setBool(gripper_client_, false);
  EXPECT_EQ(g->message, "open gripper");
  // The reply comes once the frame is written; the fake arm reads it in its own thread
  ASSERT_TRUE(waitUntil([this] {return frames(protocol::SET_GRIPPER_STATE) == 2;}));
  const auto gripper = arm_->received(protocol::SET_GRIPPER_STATE);
  ASSERT_EQ(gripper.size(), 2u);
  EXPECT_EQ(gripper[0].data, (mycobot::fake::Bytes{1, 50}));
  EXPECT_EQ(gripper[1].data, (mycobot::fake::Bytes{0, 50}));

  const auto r = trigger(release_client_);
  ASSERT_TRUE(r);
  EXPECT_TRUE(r->success);
  EXPECT_EQ(r->message, "release servos");
  EXPECT_TRUE(waitUntil([this] {return frames(protocol::RELEASE_ALL_SERVOS) == 1;}));

  std_msgs::msg::ColorRGBA colour;
  colour.r = 2.0f;    // clamped to 1
  colour.g = 0.5f;
  colour.b = -1.0f;   // clamped to 0
  led_pub_->publish(colour);
  ASSERT_TRUE(waitUntil([this] {return frames(protocol::SET_COLOR) == 1;}));
  EXPECT_EQ(arm_->received(protocol::SET_COLOR)[0].data, (mycobot::fake::Bytes{255, 128, 0}));
}

TEST_F(ArmDriverTest, LostPortFailsTheServiceAndKeepsRunning)
{
  start();
  ASSERT_TRUE(
    waitUntil(
      [this] {
        std::lock_guard<std::mutex> lock(mutex_);
        return !joints_.empty();
      }));
  arm_.reset();   // the cable is gone: every write fails with EIO
  std::this_thread::sleep_for(300ms);   // the serial loop logs its errors and goes on
  const auto r = trigger(power_client_);
  ASSERT_TRUE(r);
  EXPECT_FALSE(r->success);
  EXPECT_EQ(r->message.rfind("power on failed: ", 0), 0u) << r->message;
}
