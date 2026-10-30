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

// JoystickTeleop with a FIFO as the joystick and a fake arm_driver in the same executor: it
// publishes /joint_states and /arm/control_owner, records /arm/command and /arm/led, and
// answers take_control (as ControlLease does), power_on and set_gripper.

#include <fcntl.h>
#include <linux/joystick.h>
#include <sys/stat.h>
#include <unistd.h>

#include <gtest/gtest.h>
#include <optional>   // cpplint takes it for a C header, as gtest.h: both before the C++ headers

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "braco_teleop/joystick_teleop.hpp"

using namespace std::chrono_literals;
using braco_interfaces::msg::ArmCommand;
using braco_interfaces::srv::TakeControl;
namespace tl = mycobot::teleop;

namespace
{

constexpr uint8_t AXIS_J1 = 0;    // left stick X (inverted: + moves J1 down)
constexpr uint8_t AXIS_J2 = 1;    // left stick Y
constexpr uint8_t TRIGGER = 4;    // never a joint
constexpr int16_t FULL = 32767;
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

// A joystick device: a FIFO the test writes js_event structs into. Opened read-write, so the
// node's non-blocking open always finds a writer. `create` false: only the path, made later.
class FifoJoystick
{
public:
  explicit FifoJoystick(bool create = true)
  {
    char dir[] = "/tmp/teleop_jsXXXXXX";
    if (mkdtemp(dir) == nullptr) {return;}
    dir_ = dir;
    path_ = dir_ + "/js0";
    if (create) {make();}
  }

  ~FifoJoystick()
  {
    if (fd_ >= 0) {close(fd_);}
    unlink(path_.c_str());
    rmdir(dir_.c_str());
  }

  FifoJoystick(const FifoJoystick &) = delete;
  FifoJoystick & operator=(const FifoJoystick &) = delete;

  void make()
  {
    mkfifo(path_.c_str(), 0600);
    fd_ = open(path_.c_str(), O_RDWR | O_NONBLOCK);
  }

  const std::string & path() const {return path_;}

  // `ms`: the event's time; by default a second after the previous one (outside every guard).
  void axis(uint8_t number, int16_t value, std::optional<uint32_t> ms = {})
  {
    send(JS_EVENT_AXIS, number, value, ms);
  }
  void button(uint8_t number, int16_t value, std::optional<uint32_t> ms = {})
  {
    send(JS_EVENT_BUTTON, number, value, ms);
  }
  void click(uint8_t number)
  {
    button(number, 1);
    button(number, 0);
  }
  uint32_t time() const {return time_;}

  // Events between batch() and flush() go in one write, so the node reads them in one tick.
  void batch() {batching_ = true;}
  void flush()
  {
    batching_ = false;
    [[maybe_unused]] const ssize_t n =
      write(fd_, pending_.data(), pending_.size() * sizeof(js_event));
    pending_.clear();
  }

private:
  std::string dir_;
  std::string path_;
  int fd_ = -1;
  uint32_t time_ = 0;
  bool batching_ = false;
  std::vector<js_event> pending_;

  void send(uint8_t type, uint8_t number, int16_t value, std::optional<uint32_t> ms)
  {
    time_ = ms ? *ms : time_ + 1000;
    const js_event e{time_, value, type, number};
    if (batching_) {
      pending_.push_back(e);
      return;
    }
    [[maybe_unused]] const ssize_t n = write(fd_, &e, sizeof(e));
  }
};

// arm_driver as seen by the teleop node. Everything it records is under `mutex`.
class FakeDriver : public rclcpp::Node
{
public:
  struct Options
  {
    bool services = true;    // offer take_control, power_on and set_gripper
    bool states = true;      // publish /joint_states every 20 ms
    bool follow = false;     // the angles follow the commands at once
  };

  explicit FakeDriver(const Options & options)
  : Node("joystick_teleop_test_driver"), options_(options)
  {
    owner_pub_ = create_publisher<std_msgs::msg::String>(
      "arm/control_owner", rclcpp::QoS(1).transient_local());
    joint_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    command_sub_ = create_subscription<ArmCommand>(
      "arm/command", 10, [this](const ArmCommand & msg) {onCommand(msg);});
    led_sub_ = create_subscription<std_msgs::msg::ColorRGBA>(
      "arm/led", 10, [this](const std_msgs::msg::ColorRGBA & msg) {
        std::lock_guard<std::mutex> lock(mutex);
        leds.push_back(msg);
      });
    if (options_.services) {
      take_srv_ = create_service<TakeControl>(
        "arm/take_control",
        [this](TakeControl::Request::ConstSharedPtr req, TakeControl::Response::SharedPtr res) {
          std::lock_guard<std::mutex> lock(mutex);
          takes.push_back(*req);
          res->success = owner.empty() || owner == req->source || req->force;
          if (res->success) {
            owner = req->source;
            publishOwnerLocked();
          }
          res->owner = owner;
          res->message = res->success ? "" : "control held by " + owner;
        });
      power_srv_ = create_service<std_srvs::srv::Trigger>(
        "arm/power_on",
        [this](std_srvs::srv::Trigger::Request::ConstSharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr res) {
          std::lock_guard<std::mutex> lock(mutex);
          ++power_ons;
          res->success = true;
        });
      gripper_srv_ = create_service<std_srvs::srv::SetBool>(
        "arm/set_gripper",
        [this](std_srvs::srv::SetBool::Request::ConstSharedPtr req,
        std_srvs::srv::SetBool::Response::SharedPtr res) {
          std::lock_guard<std::mutex> lock(mutex);
          grippers.push_back(req->data);
          res->success = true;
        });
    }
    if (options_.states) {
      timer_ = create_wall_timer(20ms, [this]() {publishStates();});
    }
  }

  // The owner, published on /arm/control_owner (latched) like arm_driver does.
  void setOwner(const std::string & name)
  {
    std::lock_guard<std::mutex> lock(mutex);
    owner = name;
    publishOwnerLocked();
  }

  void setAngle(int joint, double deg)
  {
    std::lock_guard<std::mutex> lock(mutex);
    angles[joint - 1] = deg;
  }

  void publish(const sensor_msgs::msg::JointState & msg) {joint_pub_->publish(msg);}

  std::mutex mutex;
  std::string owner;
  mycobot::Angles angles{};
  std::vector<ArmCommand> commands;
  std::vector<std_msgs::msg::ColorRGBA> leds;
  std::vector<TakeControl::Request> takes;
  std::vector<bool> grippers;
  int power_ons = 0;

private:
  void publishOwnerLocked()
  {
    std_msgs::msg::String msg;
    msg.data = owner;
    owner_pub_->publish(msg);
  }

  void publishStates()
  {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    std::lock_guard<std::mutex> lock(mutex);
    for (size_t i = 0; i < angles.size(); ++i) {
      msg.name.push_back("joint" + std::to_string(i + 1));
      msg.position.push_back(angles[i] * DEG_TO_RAD);
    }
    joint_pub_->publish(msg);
  }

  void onCommand(const ArmCommand & msg)
  {
    std::lock_guard<std::mutex> lock(mutex);
    commands.push_back(msg);
    if (!options_.follow) {return;}
    if (msg.type == ArmCommand::ALL_ANGLES) {
      std::copy(msg.angles_deg.begin(), msg.angles_deg.end(), angles.begin());
    } else if (msg.type == ArmCommand::JOINTS) {
      for (const auto & t : msg.targets) {angles[t.joint - 1] = t.angle_deg;}
    }
  }

  Options options_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr owner_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_pub_;
  rclcpp::Subscription<ArmCommand>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::ColorRGBA>::SharedPtr led_sub_;
  rclcpp::Service<TakeControl>::SharedPtr take_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr power_srv_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr gripper_srv_;
  rclcpp::TimerBase::SharedPtr timer_;
};

class JoystickTeleopTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
  static void TearDownTestSuite() {rclcpp::shutdown();}

  void TearDown() override
  {
    if (executor_) {executor_->cancel();}
    if (spinner_.joinable()) {spinner_.join();}
  }

  // The fake driver, then the teleop node on `device` (the FIFO by default).
  void start(
    const FakeDriver::Options & options = {}, const std::vector<rclcpp::Parameter> & extra = {},
    const std::string & device = "")
  {
    driver_ = std::make_shared<FakeDriver>(options);
    std::vector<rclcpp::Parameter> params = {
      rclcpp::Parameter("device", device.empty() ? js_.path() : device)};
    params.insert(params.end(), extra.begin(), extra.end());
    teleop_ = std::make_shared<braco_teleop::JoystickTeleop>(
      rclcpp::NodeOptions().parameter_overrides(params));

    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(driver_);
    executor_->add_node(teleop_);
    spinner_ = std::thread([this] {executor_->spin();});
    // The graph is matched both ways and the teleop node has fresh angles
    ASSERT_TRUE(
      waitUntil(
        [this] {
          return driver_->count_publishers("arm/command") > 0 &&
                 driver_->count_subscribers("joint_states") > 0 &&
                 driver_->count_subscribers("arm/control_owner") > 0;
        }));
    std::this_thread::sleep_for(100ms);
  }

  // The joystick holds the control (as after a granted take_control).
  void own()
  {
    driver_->setOwner("joystick");
    std::this_thread::sleep_for(100ms);
  }

  size_t count(uint8_t type)
  {
    std::lock_guard<std::mutex> lock(driver_->mutex);
    size_t n = 0;
    for (const auto & c : driver_->commands) {n += c.type == type ? 1 : 0;}
    return n;
  }

  // The first JOINTS command at or after `from` with a target for `joint`.
  std::optional<braco_interfaces::msg::JointTarget> target(int joint, size_t from = 0)
  {
    std::lock_guard<std::mutex> lock(driver_->mutex);
    for (size_t i = from; i < driver_->commands.size(); ++i) {
      const auto & c = driver_->commands[i];
      if (c.type != ArmCommand::JOINTS) {continue;}
      for (const auto & t : c.targets) {
        if (t.joint == joint) {return t;}
      }
    }
    return std::nullopt;
  }

  bool waitTarget(int joint, size_t from = 0, std::chrono::milliseconds timeout = 3s)
  {
    return waitUntil([&] {return target(joint, from).has_value();}, timeout);
  }

  size_t commandCount()
  {
    std::lock_guard<std::mutex> lock(driver_->mutex);
    return driver_->commands.size();
  }

  // Wait until no command came for `quiet`, then return how many came.
  size_t settle(std::chrono::milliseconds quiet = 200ms)
  {
    size_t n = commandCount();
    while (true) {
      std::this_thread::sleep_for(quiet);
      const size_t now = commandCount();
      if (now == n) {return n;}
      n = now;
    }
  }

  std::optional<std_msgs::msg::ColorRGBA> lastLed()
  {
    std::lock_guard<std::mutex> lock(driver_->mutex);
    if (driver_->leds.empty()) {return std::nullopt;}
    return driver_->leds.back();
  }

  size_t takeCount()
  {
    std::lock_guard<std::mutex> lock(driver_->mutex);
    return driver_->takes.size();
  }

  FifoJoystick js_;
  std::shared_ptr<FakeDriver> driver_;
  std::shared_ptr<braco_teleop::JoystickTeleop> teleop_;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
  std::thread spinner_;
};

}  // namespace

// ---- Joystick connection --------------------------------------------------------

TEST_F(JoystickTeleopTest, MissingJoystickIsOpenedOnceItAppears)
{
  FifoJoystick later(false);
  start({}, {}, later.path());
  std::this_thread::sleep_for(300ms);
  EXPECT_FALSE(lastLed());   // nothing connected yet
  later.make();
  // Retried every second; on connecting the LED shows the start mode (green)
  ASSERT_TRUE(waitUntil([this] {return lastLed().has_value();}, 2500ms));
  EXPECT_FLOAT_EQ(lastLed()->g, 1.0f);
  EXPECT_FLOAT_EQ(lastLed()->r, 0.0f);
}

TEST_F(JoystickTeleopTest, AJoystickThatFailsToReadIsOpenedAgain)
{
  // A directory opens, but reading it fails (EISDIR): treated as unplugged, reopened every second
  char dir[] = "/tmp/teleop_dirXXXXXX";
  ASSERT_NE(mkdtemp(dir), nullptr);
  start({}, {}, dir);
  EXPECT_TRUE(
    waitUntil(
      [this] {
        std::lock_guard<std::mutex> lock(driver_->mutex);
        return driver_->leds.size() >= 2;
      }, 3s));
  rmdir(dir);
}

// ---- Control --------------------------------------------------------------------

TEST_F(JoystickTeleopTest, FirstStickMoveTakesTheControlAndJogs)
{
  start();
  js_.axis(TRIGGER, FULL);   // ignored
  js_.axis(AXIS_J2, FULL);
  ASSERT_TRUE(waitTarget(2));
  {
    std::lock_guard<std::mutex> lock(driver_->mutex);
    ASSERT_EQ(driver_->takes.size(), 1u);
    EXPECT_EQ(driver_->takes[0].source, "joystick");
    EXPECT_FALSE(driver_->takes[0].force);
    EXPECT_EQ(driver_->commands.back().source, "joystick");
  }
  // A step ahead of the real angle (0), at most the mode's lead (30 %: 10 deg)
  const auto t = target(2);
  EXPECT_GT(t->angle_deg, 0.0);
  EXPECT_LE(t->angle_deg, tl::SPEED_MODES[tl::START_MODE].leadMax);
  EXPECT_TRUE(
    waitUntil(
      [this] {
        std::lock_guard<std::mutex> lock(driver_->mutex);
        return driver_->power_ons == 1;
      }));

  js_.axis(AXIS_J2, 0);
  ASSERT_TRUE(waitUntil([this] {return count(ArmCommand::STOP) == 1;}));
  settle();
  EXPECT_EQ(count(ArmCommand::STOP), 1u);   // stop once, then idle
}

TEST_F(JoystickTeleopTest, HeldByAnotherClientUntilTheForceButtonIsHeld)
{
  start({}, {rclcpp::Parameter("force_hold_s", 0.3)});
  driver_->setOwner("web");
  // The LED is dimmed while the web page has the control
  ASSERT_TRUE(waitUntil([this] {return lastLed() && lastLed()->g < 0.5f;}));
  EXPECT_FLOAT_EQ(lastLed()->g, 0.15f);

  js_.axis(AXIS_J2, FULL);
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(takeCount(), 0u);
  EXPECT_EQ(commandCount(), 0u);
  js_.axis(AXIS_J2, 0);

  js_.button(tl::BUTTON_FORCE_CONTROL, 1);
  ASSERT_TRUE(waitUntil([this] {return takeCount() == 1;}));
  {
    std::lock_guard<std::mutex> lock(driver_->mutex);
    EXPECT_TRUE(driver_->takes[0].force);
  }
  EXPECT_TRUE(waitUntil([this] {return lastLed() && lastLed()->g == 1.0f;}));
  std::this_thread::sleep_for(500ms);
  EXPECT_EQ(takeCount(), 1u);   // fires once per press
  js_.button(tl::BUTTON_FORCE_CONTROL, 0);
}

TEST_F(JoystickTeleopTest, RefusedTakeLearnsTheOwner)
{
  start();
  {
    std::lock_guard<std::mutex> lock(driver_->mutex);
    driver_->owner = "web";   // not published: the joystick still thinks it is free
  }
  js_.axis(AXIS_J2, FULL);
  ASSERT_TRUE(waitUntil([this] {return takeCount() == 1;}));
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(takeCount(), 1u);   // the answer named the owner: no new request
  EXPECT_EQ(commandCount(), 0u);
}

TEST_F(JoystickTeleopTest, LosingTheControlForgetsTheJogWithoutStop)
{
  start();
  js_.axis(AXIS_J1, FULL);
  ASSERT_TRUE(waitTarget(1));

  // Freed while jogging (arm_driver's idle timeout): forgotten, no STOP
  driver_->setOwner("");
  size_t stops = count(ArmCommand::STOP);
  js_.axis(AXIS_J1, 0);
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count(ArmCommand::STOP), stops);

  // The next touch takes it again; then the web page takes it while the stick is held
  size_t from = commandCount();
  js_.axis(AXIS_J1, FULL);
  ASSERT_TRUE(waitUntil([this] {return takeCount() == 2;}));
  ASSERT_TRUE(waitTarget(1, from));
  driver_->setOwner("web");
  settle();
  stops = count(ArmCommand::STOP);
  from = commandCount();
  js_.axis(AXIS_J1, 0);   // the web page's move is not cut
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(count(ArmCommand::STOP), stops);
  EXPECT_EQ(commandCount(), from);
}

TEST_F(JoystickTeleopTest, WithoutServicesNothingIsCalled)
{
  start({false, true, false});
  js_.axis(AXIS_J2, FULL);   // cannot take the control
  std::this_thread::sleep_for(300ms);
  EXPECT_EQ(commandCount(), 0u);
  js_.axis(AXIS_J2, 0);

  own();   // as if taken earlier: jogging needs no service, the gripper and power_on do
  js_.click(tl::BUTTON_GRIPPER_CLOSE);
  js_.axis(AXIS_J2, FULL);
  EXPECT_TRUE(waitTarget(2));
}

// ---- Buttons --------------------------------------------------------------------

TEST_F(JoystickTeleopTest, GripperButtons)
{
  start();
  own();
  js_.click(tl::BUTTON_GRIPPER_CLOSE);
  ASSERT_TRUE(
    waitUntil(
      [this] {
        std::lock_guard<std::mutex> lock(driver_->mutex);
        return driver_->grippers.size() == 1;
      }));
  js_.click(tl::BUTTON_GRIPPER_OPEN);
  ASSERT_TRUE(
    waitUntil(
      [this] {
        std::lock_guard<std::mutex> lock(driver_->mutex);
        return driver_->grippers.size() == 2;
      }));
  std::lock_guard<std::mutex> lock(driver_->mutex);
  EXPECT_TRUE(driver_->grippers[0]);
  EXPECT_FALSE(driver_->grippers[1]);
}

TEST_F(JoystickTeleopTest, SpeedButtonCyclesTheModeAndBlocksAxesFiredWithIt)
{
  start({}, {rclcpp::Parameter("debug", true)});
  own();
  js_.click(tl::BUTTON_SPEED_MODE);   // 30 % -> 50 % (purple)
  ASSERT_TRUE(waitUntil([this] {return lastLed() && lastLed()->r > 0.4f;}));
  EXPECT_FLOAT_EQ(lastLed()->r, 128.0f / 255.0f);
  EXPECT_FLOAT_EQ(lastLed()->g, 0.0f);

  // A stick pushed while button 9 is down is blocked until it comes back to the centre
  js_.batch();
  js_.button(tl::BUTTON_SPEED_MODE, 1);
  js_.axis(AXIS_J2, FULL);
  js_.button(tl::BUTTON_SPEED_MODE, 0);
  js_.flush();
  std::this_thread::sleep_for(300ms);
  EXPECT_FALSE(target(2));
  js_.axis(AXIS_J2, 0);
  js_.axis(AXIS_J2, FULL);
  ASSERT_TRUE(waitTarget(2));
  js_.axis(AXIS_J2, 0);
  settle();

  // A stick pushed just before button 9 (within the guard) is blocked as well
  const size_t from = commandCount();
  js_.batch();
  js_.axis(AXIS_J1, FULL);
  js_.button(tl::BUTTON_SPEED_MODE, 1, js_.time() + 100);
  js_.button(tl::BUTTON_SPEED_MODE, 0);
  js_.flush();
  std::this_thread::sleep_for(300ms);
  EXPECT_FALSE(target(1, from));
}

// ---- Jogging --------------------------------------------------------------------

TEST_F(JoystickTeleopTest, IdleJointsAreHeldAndReleasedJointsPinned)
{
  start({}, {rclcpp::Parameter("debug", true)});
  own();
  js_.axis(AXIS_J1, FULL);
  js_.axis(AXIS_J2, FULL);
  ASSERT_TRUE(waitTarget(1));
  ASSERT_TRUE(waitTarget(2));

  // J4 sags while the others move: sent back to its latched angle
  size_t from = commandCount();
  driver_->setAngle(4, 5.0);
  ASSERT_TRUE(waitTarget(4, from));
  EXPECT_NEAR(target(4, from)->angle_deg, 0.0, 0.05);
  EXPECT_EQ(target(4, from)->speed, tl::HOLD_SPEED);

  // J1 let go while J2 goes on: pinned where it is
  from = commandCount();
  js_.axis(AXIS_J1, 0);
  ASSERT_TRUE(waitTarget(1, from));
  EXPECT_EQ(target(1, from)->speed, tl::HOLD_SPEED);
}

TEST_F(JoystickTeleopTest, AJointAtItsLimitIsNotPushedFurther)
{
  start();
  driver_->setAngle(2, mycobot::JOINT_LIMITS[1].max);
  own();
  js_.axis(AXIS_J2, FULL);
  std::this_thread::sleep_for(300ms);
  EXPECT_FALSE(target(2));
  js_.axis(AXIS_J2, -FULL);   // away from the limit: allowed
  EXPECT_TRUE(waitTarget(2));
  EXPECT_LT(target(2)->angle_deg, mycobot::JOINT_LIMITS[1].max);
}

TEST_F(JoystickTeleopTest, NoFreshAnglesStopsInsteadOfJogging)
{
  start({true, false, false});
  own();
  js_.axis(AXIS_J2, FULL);
  EXPECT_TRUE(waitUntil([this] {return count(ArmCommand::STOP) > 0;}));
  EXPECT_EQ(count(ArmCommand::JOINTS), 0u);
}

TEST_F(JoystickTeleopTest, JointStatesAreReadByName)
{
  start({true, false, false});
  own();
  // Names in reverse order: J2 is the fifth position. Then two messages that are ignored:
  // too few positions, and a name past the end of the positions.
  sensor_msgs::msg::JointState reversed;
  reversed.name = {"joint6", "joint5", "joint4", "joint3", "joint2", "joint1"};
  reversed.position = {0.0, 0.0, 0.0, 0.0, 50.0 * DEG_TO_RAD, 0.0};
  sensor_msgs::msg::JointState short_msg;
  short_msg.position = {0.0, 0.0, 0.0, 0.0, 0.0};
  sensor_msgs::msg::JointState past_end;
  past_end.name = {"a", "b", "c", "d", "e", "f", "joint1"};
  past_end.position = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

  std::atomic<bool> feeding{true};
  std::thread feeder([&] {
      while (feeding) {
        driver_->publish(reversed);
        driver_->publish(short_msg);
        driver_->publish(past_end);
        std::this_thread::sleep_for(20ms);
      }
    });
  js_.axis(AXIS_J2, FULL);
  const bool jogged = waitTarget(2);
  feeding = false;
  feeder.join();
  ASSERT_TRUE(jogged);
  EXPECT_GT(target(2)->angle_deg, 50.0);
}

// ---- Homing ---------------------------------------------------------------------

TEST_F(JoystickTeleopTest, HomingStepsToZeroAndStops)
{
  start({true, true, true});
  driver_->setAngle(1, 40.0);
  driver_->setAngle(3, -30.0);
  own();
  js_.click(tl::BUTTON_HOME);
  ASSERT_TRUE(waitUntil([this] {return count(ArmCommand::ALL_ANGLES) > 0;}));
  {
    std::lock_guard<std::mutex> lock(driver_->mutex);
    EXPECT_EQ(driver_->commands.back().speed, tl::HOME_SPEED);
  }
  ASSERT_TRUE(
    waitUntil(
      [this] {
        std::lock_guard<std::mutex> lock(driver_->mutex);
        return std::abs(driver_->angles[0]) < 1.0 && std::abs(driver_->angles[2]) < 1.0;
      }));
  const size_t n = settle();
  EXPECT_EQ(count(ArmCommand::STOP), 0u);
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(commandCount(), n);   // home reached: nothing more
}

TEST_F(JoystickTeleopTest, HomingIsCancelledBySecondClick)
{
  start();
  driver_->setAngle(1, 40.0);
  own();
  js_.click(tl::BUTTON_HOME);
  ASSERT_TRUE(waitUntil([this] {return count(ArmCommand::ALL_ANGLES) > 0;}));
  js_.click(tl::BUTTON_HOME);
  ASSERT_TRUE(waitUntil([this] {return count(ArmCommand::STOP) == 1;}));
  const size_t n = settle();
  std::this_thread::sleep_for(200ms);
  EXPECT_EQ(commandCount(), n);
}

TEST_F(JoystickTeleopTest, HomingIsAbortedByAStick)
{
  start();
  driver_->setAngle(1, 40.0);
  own();
  js_.click(tl::BUTTON_HOME);
  ASSERT_TRUE(waitUntil([this] {return count(ArmCommand::ALL_ANGLES) > 0;}));
  js_.axis(AXIS_J2, FULL);
  ASSERT_TRUE(waitUntil([this] {return count(ArmCommand::STOP) == 1;}));
  EXPECT_TRUE(waitTarget(2));   // then it jogs
}

TEST_F(JoystickTeleopTest, HomingWithoutFeedbackGivesUp)
{
  start({true, false, false});
  own();
  js_.click(tl::BUTTON_HOME);
  // One stop per tick without angles, then homing ends after HOME_MAX_BAD_READS
  ASSERT_TRUE(
    waitUntil(
      [this] {
        return count(ArmCommand::STOP) == static_cast<size_t>(tl::HOME_MAX_BAD_READS);
      }));
  settle();
  EXPECT_EQ(count(ArmCommand::STOP), static_cast<size_t>(tl::HOME_MAX_BAD_READS));
  EXPECT_EQ(count(ArmCommand::ALL_ANGLES), 0u);
}
