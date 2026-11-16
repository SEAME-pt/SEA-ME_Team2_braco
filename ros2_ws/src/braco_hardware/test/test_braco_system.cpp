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

// BracoSystem (#87) loaded by pluginlib from the real-arm URDF, against arm_driver on PtyArm
// (mycobot_cpp/test/pty_arm.hpp): read() must give the fake arm's angles, in radians.
// write() (#101) against arm_driver's topics, faked: it sends ALL_ANGLES as mission only while
// mission owns the arm, and aborts the goal on the STOP latch (ADR-0005) or a lost control.

#include <gtest/gtest.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_prefix.hpp"
#include "ament_index_cpp/get_package_share_path.hpp"
#include "braco_interfaces/msg/arm_command.hpp"
#include "hardware_interface/component_parser.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_component_params.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "pluginlib/class_loader.hpp"
#include "pty_arm.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using braco_interfaces::msg::ArmCommand;
using mycobot::fake::PtyArm;
using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace
{

// The URDF of braco.launch.py sim:=false: braco.urdf.xacro with real_arm:=true, through xacro.
std::string realArmUrdf()
{
  const auto xacro_file =
    ament_index_cpp::get_package_share_path("braco_description") / "urdf" / "braco.urdf.xacro";
  const std::string command = "xacro " + xacro_file.string() + " real_arm:=true";
  std::unique_ptr<FILE, int (*)(FILE *)> pipe(popen(command.c_str(), "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("cannot run: " + command);
  }
  std::string urdf;
  std::array<char, 4096> buffer{};
  while (fgets(buffer.data(), static_cast<int>(buffer.size()), pipe.get()) != nullptr) {
    urdf += buffer.data();
  }
  return urdf;
}

// The <ros2_control> block named BracoSystem in that URDF.
hardware_interface::HardwareInfo bracoSystemInfo()
{
  for (const auto & info : hardware_interface::parse_control_resources_from_urdf(realArmUrdf())) {
    if (info.name == "BracoSystem") {
      return info;
    }
  }
  throw std::runtime_error("no BracoSystem <ros2_control> block in the real-arm URDF");
}

// The installed arm_driver, on `port`, as a child process: SIGINT on destruction (its last stop()
// still reaches the port), SIGKILL if it hangs, so a failed assertion leaves no orphan.
class ArmDriverProcess
{
public:
  explicit ArmDriverProcess(const std::string & port)
  {
    std::filesystem::path prefix;
    ament_index_cpp::get_package_prefix("braco_driver", prefix);
    const std::string exe = (prefix / "lib" / "braco_driver" / "arm_driver").string();
    const std::string port_arg = "port:=" + port;
    std::array<const char *, 5> argv{exe.c_str(), "--ros-args", "-p", port_arg.c_str(), nullptr};
    if (posix_spawn(
        &pid_, exe.c_str(), nullptr, nullptr, const_cast<char * const *>(argv.data()),
        environ) != 0)
    {
      throw std::runtime_error("cannot spawn " + exe);
    }
  }

  ~ArmDriverProcess()
  {
    kill(pid_, SIGINT);
    const auto deadline = std::chrono::steady_clock::now() + 3s;
    while (std::chrono::steady_clock::now() < deadline) {
      if (waitpid(pid_, nullptr, WNOHANG) == pid_) {
        return;
      }
      std::this_thread::sleep_for(20ms);
    }
    kill(pid_, SIGKILL);
    waitpid(pid_, nullptr, 0);
  }

  ArmDriverProcess(const ArmDriverProcess &) = delete;
  ArmDriverProcess & operator=(const ArmDriverProcess &) = delete;

private:
  pid_t pid_{};
};

// joint1..`joints` at 0.1 rad each, as arm_driver would publish them (no velocity)
sensor_msgs::msg::JointState jointStates(int joints)
{
  sensor_msgs::msg::JointState msg;
  for (int i = 1; i <= joints; ++i) {
    msg.name.push_back("joint" + std::to_string(i));
    msg.position.push_back(0.1);
  }
  return msg;
}

// Publishes `msg` on /joint_states every 50 ms until destroyed. Repeating it, not publishing
// once, means the plugin's subscription gets it however long discovery takes.
class JointStatesPublisher
{
public:
  explicit JointStatesPublisher(const sensor_msgs::msg::JointState & msg)
  : node_(std::make_shared<rclcpp::Node>("test_joint_states")),
    publisher_(node_->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10)),
    thread_([this, msg] {
        while (!stop_) {
          publisher_->publish(msg);
          std::this_thread::sleep_for(50ms);
        }
      })
  {}

  ~JointStatesPublisher()
  {
    stop_ = true;
    thread_.join();
  }

  JointStatesPublisher(const JointStatesPublisher &) = delete;
  JointStatesPublisher & operator=(const JointStatesPublisher &) = delete;

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  std::atomic<bool> stop_{false};
  std::thread thread_;   // last: starts after the members it uses
};

// arm_driver's side of write(), faked: /arm/control_owner and /arm/stopped latched, as arm_driver
// publishes them, and every ArmCommand that write() sends on /arm/command.
class ArmDriverTopics
{
public:
  ArmDriverTopics()
  : node_(std::make_shared<rclcpp::Node>("test_arm_driver_topics")),
    owner_pub_(node_->create_publisher<std_msgs::msg::String>(
        "/arm/control_owner", rclcpp::QoS(1).transient_local())),
    stopped_pub_(node_->create_publisher<std_msgs::msg::Bool>(
        "/arm/stopped", rclcpp::QoS(1).transient_local())),
    command_sub_(node_->create_subscription<ArmCommand>(
        "/arm/command", 10, [this](const ArmCommand & msg) {
          std::lock_guard<std::mutex> lock(mutex_);
          commands_.push_back(msg);
        }))
  {
    executor_.add_node(node_);
    thread_ = std::thread([this] {executor_.spin();});
  }

  ~ArmDriverTopics()
  {
    executor_.cancel();
    thread_.join();
  }

  ArmDriverTopics(const ArmDriverTopics &) = delete;
  ArmDriverTopics & operator=(const ArmDriverTopics &) = delete;

  void owner(const std::string & who)
  {
    std_msgs::msg::String msg;
    msg.data = who;
    owner_pub_->publish(msg);
  }

  void stopped(bool latched)
  {
    std_msgs::msg::Bool msg;
    msg.data = latched;
    stopped_pub_->publish(msg);
  }

  std::vector<ArmCommand> commands()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return commands_;
  }

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr owner_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr stopped_pub_;
  rclcpp::Subscription<ArmCommand>::SharedPtr command_sub_;
  std::mutex mutex_;
  std::vector<ArmCommand> commands_;
  rclcpp::executors::SingleThreadedExecutor executor_;
  std::thread thread_;
};

constexpr int64_t MS = 1000000;   // nanoseconds
// Targets of arm_controller, distinct per joint so a wrong joint or unit shows
constexpr std::array<double, 6> TARGETS_RAD = {0.1, -0.2, 0.3, -0.4, 0.5, -0.6};

class BracoSystemTest : public ::testing::Test
{
protected:
  // Domain 28, reserved for this test (docker/config/team-domains.tsv), instead of the runner's
  // random one, which can be the real arm's (22); discovery kept on this host. arm_driver
  // inherits both through environ.
  static void SetUpTestSuite()
  {
    setenv("ROS_DOMAIN_ID", "28", 1);
    setenv("ROS_AUTOMATIC_DISCOVERY_RANGE", "LOCALHOST", 1);
    rclcpp::init(0, nullptr);
  }
  static void TearDownTestSuite() {rclcpp::shutdown();}

  // The plugin as ros2_control_node loads it: by name, init() with our executor, its state
  // interfaces exported. The executor spins in its own thread, like the controller manager's.
  void SetUp() override
  {
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    system_ = loader_.createUniqueInstance("braco_hardware/BracoSystem");
    ASSERT_EQ(system_->init(params(bracoSystemInfo(), executor_)), CallbackReturn::SUCCESS);
    system_->on_export_state_interfaces();
    system_->on_export_command_interfaces();

    spinner_ = std::thread([this] {executor_->spin();});
  }

  // init()'s parameters; with no executor, init() creates no node for the plugin
  static hardware_interface::HardwareComponentParams params(
    const hardware_interface::HardwareInfo & info, const rclcpp::Executor::SharedPtr & executor)
  {
    hardware_interface::HardwareComponentParams params;
    params.hardware_info = info;
    params.clock = std::make_shared<rclcpp::Clock>(RCL_STEADY_TIME);
    params.logger = rclcpp::get_logger("test_braco_system");
    params.executor = executor;
    return params;
  }

  void TearDown() override
  {
    executor_->cancel();
    if (spinner_.joinable()) {spinner_.join();}
    system_.reset();   // before loader_, which owns the plugin library
  }

  using State = lifecycle_msgs::msg::State;

  // The lifecycle callbacks, called directly with the state the plugin comes from
  CallbackReturn configure()
  {
    return system_->on_configure(
      rclcpp_lifecycle::State(State::PRIMARY_STATE_UNCONFIGURED, "unconfigured"));
  }
  CallbackReturn activate()
  {
    return system_->on_activate(
      rclcpp_lifecycle::State(State::PRIMARY_STATE_INACTIVE, "inactive"));
  }
  return_type read() {return system_->read(rclcpp::Time(0, 0, RCL_STEADY_TIME), 10ms);}

  // write() at the next tick of the manager's clock: 60 ms later, above its 20 Hz throttle. The
  // clock is ours, so HOLD_S passes without waiting for it.
  return_type write()
  {
    now_ns_ += 60 * MS;
    return system_->write(rclcpp::Time(now_ns_, RCL_STEADY_TIME), 10ms);
  }

  // write() every tick, 20 ms apart in real time so the topics flow, until `done(result)` or
  // `timeout`; returns the last result
  template<typename Done>
  return_type writeUntil(Done done, std::chrono::milliseconds timeout = 5000ms)
  {
    return_type result = return_type::OK;
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      result = write();
      if (done(result)) {break;}
      std::this_thread::sleep_for(20ms);
    }
    return result;
  }

  // write() for `duration`; true if every result was OK
  bool writeFor(std::chrono::milliseconds duration)
  {
    bool all_ok = true;
    writeUntil(
      [&](return_type r) {
        all_ok = all_ok && r == return_type::OK;
        return false;
      }, duration);
    return all_ok;
  }

  // The targets arm_controller writes to our position command interfaces
  void setTargets(const std::array<double, 6> & rad)
  {
    for (size_t i = 0; i < rad.size(); ++i) {
      system_->set_command("joint" + std::to_string(i + 1) + "/position", rad[i]);
    }
  }

  static std::vector<std::string> positionInterfaces()
  {
    std::vector<std::string> names;
    for (int i = 1; i <= 6; ++i) {names.push_back("joint" + std::to_string(i) + "/position");}
    return names;
  }

  // arm_controller claims / releases our position commands, as the controller manager tells us
  void claim()
  {
    ASSERT_EQ(system_->perform_command_mode_switch(positionInterfaces(), {}), return_type::OK);
  }
  void release()
  {
    ASSERT_EQ(system_->perform_command_mode_switch({}, positionInterfaces()), return_type::OK);
  }

  // Active, arm_controller holding TARGETS_RAD, mission owning the arm: write() is sending
  void startSending(ArmDriverTopics & topics)
  {
    ASSERT_EQ(configure(), CallbackReturn::SUCCESS);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);   // needs /joint_states
    setTargets(TARGETS_RAD);
    claim();
    topics.owner("mission");
    writeUntil([&](return_type) {return !topics.commands().empty();});
    ASSERT_FALSE(topics.commands().empty()) << "write() sent nothing while mission owns the arm";
  }

  // Commands that arrive within 300 ms
  static size_t settledCount(ArmDriverTopics & topics)
  {
    std::this_thread::sleep_for(300ms);
    return topics.commands().size();
  }

  int64_t now_ns_{1000 * MS};

  pluginlib::ClassLoader<hardware_interface::SystemInterface> loader_{
    "hardware_interface", "hardware_interface::SystemInterface"};
  pluginlib::UniquePtr<hardware_interface::SystemInterface> system_;
  rclcpp::Executor::SharedPtr executor_;
  std::thread spinner_;
};

}  // namespace

// Nothing publishes /joint_states: read() must refuse, not report stale or zero joints.
TEST_F(BracoSystemTest, ReadFailsBeforeFirstJointStates)
{
  ASSERT_EQ(configure(), CallbackReturn::SUCCESS);
  EXPECT_EQ(read(), return_type::ERROR);
}

// No arm_driver: on_activate waits its 2 s for the first /joint_states, then fails.
TEST_F(BracoSystemTest, ActivateFailsWithoutJointStates)
{
  ASSERT_EQ(configure(), CallbackReturn::SUCCESS);
  const auto start = std::chrono::steady_clock::now();
  EXPECT_EQ(activate(), CallbackReturn::ERROR);
  EXPECT_GE(std::chrono::steady_clock::now() - start, 1900ms);
}

// arm_driver on PtyArm publishes /joint_states: read() gives each URDF joint its own angle, in
// radians, and velocity 0 (arm_driver publishes none). Distinct angles catch a wrong joint.
TEST_F(BracoSystemTest, ReadGivesArmDriverAnglesInRadians)
{
  const mycobot::Angles angles = {10, -20, 30, -40, 50, -60};
  PtyArm arm;
  arm.update([&](auto & s) {s.angles = angles;});
  arm.serve();
  ArmDriverProcess driver(arm.path());   // after arm: stopped before the port closes

  // The driver's start-up stays out of on_activate's 2 s
  ASSERT_TRUE(arm.waitFor(mycobot::protocol::GET_ANGLES, 10s));
  ASSERT_EQ(configure(), CallbackReturn::SUCCESS);
  ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
  ASSERT_EQ(read(), return_type::OK);

  for (size_t i = 0; i < angles.size(); ++i) {
    const std::string joint = "joint" + std::to_string(i + 1);
    EXPECT_NEAR(system_->get_state<double>(joint + "/position"), angles[i] * M_PI / 180.0, 1e-3)
      << joint;
    EXPECT_EQ(system_->get_state<double>(joint + "/velocity"), 0.0) << joint;
  }
}

// read() fills position and velocity of every joint: a URDF joint without velocity is refused.
TEST_F(BracoSystemTest, InitFailsWithoutVelocityState)
{
  auto info = bracoSystemInfo();
  auto & states = info.joints.at(2).state_interfaces;
  states.erase(
    std::remove_if(
      states.begin(), states.end(),
      [](const auto & state) {return state.name == hardware_interface::HW_IF_VELOCITY;}),
    states.end());

  auto system = loader_.createUniqueInstance("braco_hardware/BracoSystem");
  EXPECT_EQ(system->init(params(info, nullptr)), CallbackReturn::ERROR);
}

// /joint_states without joint6: read() must refuse, not leave joint6 at its last value.
TEST_F(BracoSystemTest, ReadFailsWhenJointMissing)
{
  JointStatesPublisher publisher(jointStates(5));
  ASSERT_EQ(configure(), CallbackReturn::SUCCESS);
  ASSERT_EQ(activate(), CallbackReturn::SUCCESS);   // returns once a message arrived
  EXPECT_EQ(read(), return_type::ERROR);
}

// A second on_configure forgets the last /joint_states: read() fails until a new one arrives.
TEST_F(BracoSystemTest, ReconfigureDropsStaleJointStates)
{
  {
    JointStatesPublisher publisher(jointStates(6));
    ASSERT_EQ(configure(), CallbackReturn::SUCCESS);
    ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
    ASSERT_EQ(read(), return_type::OK);
  }
  std::this_thread::sleep_for(200ms);   // a message still in flight lands before the reconfigure
  ASSERT_EQ(configure(), CallbackReturn::SUCCESS);
  EXPECT_EQ(read(), return_type::ERROR);
}

// init() with no executor gives the plugin no node: on_configure can't subscribe, and says so.
TEST_F(BracoSystemTest, ConfigureFailsWithoutNode)
{
  auto system = loader_.createUniqueInstance("braco_hardware/BracoSystem");
  ASSERT_EQ(system->init(params(bracoSystemInfo(), nullptr)), CallbackReturn::SUCCESS);
  EXPECT_EQ(
    system->on_configure(
      rclcpp_lifecycle::State(State::PRIMARY_STATE_UNCONFIGURED, "unconfigured")),
    CallbackReturn::ERROR);
}

// ---- on_init(): the URDF's joints and <param>s ----------------------------------------------

// write() fills ArmCommand.angles_deg[i] from the i-th joint: a URDF without six joints is refused.
TEST_F(BracoSystemTest, InitFailsWithoutSixJoints)
{
  auto info = bracoSystemInfo();
  info.joints.pop_back();
  auto system = loader_.createUniqueInstance("braco_hardware/BracoSystem");
  EXPECT_EQ(system->init(params(info, nullptr)), CallbackReturn::ERROR);
}

// publish_rate_hz > 0 and speed 1..100, numbers: anything else is refused at init.
TEST_F(BracoSystemTest, InitChecksTheParameters)
{
  const auto info = bracoSystemInfo();
  const std::vector<std::pair<std::string, std::string>> bad = {
    {"speed", "0"}, {"speed", "101"}, {"speed", "fast"}, {"publish_rate_hz", "0"},
    {"publish_rate_hz", "often"}};
  for (const auto & [name, value] : bad) {
    auto with_param = info;
    with_param.hardware_parameters[name] = value;
    auto system = loader_.createUniqueInstance("braco_hardware/BracoSystem");
    EXPECT_EQ(system->init(params(with_param, nullptr)), CallbackReturn::ERROR)
      << name << "=" << value;
  }
  auto good = info;
  good.hardware_parameters["speed"] = "30";
  good.hardware_parameters["publish_rate_hz"] = "10";
  auto system = loader_.createUniqueInstance("braco_hardware/BracoSystem");
  EXPECT_EQ(system->init(params(good, nullptr)), CallbackReturn::SUCCESS);
}

// ---- write() (#101, ADR-0004, ADR-0005) -----------------------------------------------------

// mission owns the arm: arm_controller's targets go to arm_driver as ALL_ANGLES, source mission,
// in degrees, at the default speed (50).
TEST_F(BracoSystemTest, WriteSendsTheTargetsAsMissionInDegrees)
{
  JointStatesPublisher joint_states(jointStates(6));
  ArmDriverTopics topics;
  startSending(topics);

  const auto msg = topics.commands().front();
  EXPECT_EQ(msg.type, ArmCommand::ALL_ANGLES);
  EXPECT_EQ(msg.source, "mission");
  EXPECT_EQ(msg.speed, 50);
  for (size_t i = 0; i < TARGETS_RAD.size(); ++i) {
    EXPECT_NEAR(msg.angles_deg[i], TARGETS_RAD[i] * 180.0 / M_PI, 1e-4) << "joint" << i + 1;
  }
}

// Another client owns the arm (or nobody): write() sends nothing and reports no error.
TEST_F(BracoSystemTest, WriteSendsNothingWhileAnotherClientOwnsTheArm)
{
  JointStatesPublisher joint_states(jointStates(6));
  ArmDriverTopics topics;
  ASSERT_EQ(configure(), CallbackReturn::SUCCESS);
  ASSERT_EQ(activate(), CallbackReturn::SUCCESS);
  setTargets(TARGETS_RAD);
  claim();
  topics.owner("web");
  EXPECT_TRUE(writeFor(1000ms));
  EXPECT_EQ(settledCount(topics), 0u);
}

// No controller holds our commands: write() sends nothing. After arm_controller releases them,
// their last target is not resent either.
TEST_F(BracoSystemTest, WriteSendsNothingWithoutAController)
{
  JointStatesPublisher joint_states(jointStates(6));
  ArmDriverTopics topics;
  startSending(topics);

  release();
  const auto sent = settledCount(topics);
  EXPECT_TRUE(writeFor(500ms));
  EXPECT_EQ(settledCount(topics), sent);
}

// A target that stops changing is resent for HOLD_S (2 s), then not at all (arm_driver's
// watchdog stops the arm, already there); a new target is sent again.
TEST_F(BracoSystemTest, WriteHoldsAStillTargetForTwoSeconds)
{
  JointStatesPublisher joint_states(jointStates(6));
  ArmDriverTopics topics;
  startSending(topics);

  auto target = TARGETS_RAD;
  target[0] += 0.1;   // a new target: the 2 s start now
  setTargets(target);
  const int64_t changed_ns = now_ns_ + 60 * MS;
  // Sent up to HOLD_S after the change, included
  while (now_ns_ + 60 * MS <= changed_ns + 2000 * MS) {write();}
  const auto during_hold = settledCount(topics);
  while (now_ns_ < changed_ns + 4000 * MS) {write();}
  EXPECT_EQ(settledCount(topics), during_hold) << "still sent after HOLD_S";
  EXPECT_NEAR(topics.commands().back().angles_deg[0], target[0] * 180.0 / M_PI, 1e-4);

  target[0] += 0.1;
  setTargets(target);
  write();
  EXPECT_GT(settledCount(topics), during_hold) << "a new target was not sent";
}

// The STOP latch (ADR-0005) during a goal: write() returns DEACTIVATE, so ros2_control stops
// arm_controller and the goal is aborted.
TEST_F(BracoSystemTest, WriteAbortsTheGoalOnTheStopLatch)
{
  JointStatesPublisher joint_states(jointStates(6));
  ArmDriverTopics topics;
  startSending(topics);

  topics.stopped(true);
  EXPECT_EQ(
    writeUntil([](return_type r) {return r == return_type::DEACTIVATE;}), return_type::DEACTIVATE);
}

// arm_driver frees the control when it latches a STOP: the free owner may arrive before
// /arm/stopped. write() must still abort the goal, not wait for it to time out.
TEST_F(BracoSystemTest, WriteAbortsOnTheStopLatchEvenWhenTheOwnerIsFreedFirst)
{
  JointStatesPublisher joint_states(jointStates(6));
  ArmDriverTopics topics;
  startSending(topics);

  topics.owner("");
  EXPECT_TRUE(writeFor(300ms));   // a free owner alone is not an error
  topics.stopped(true);
  EXPECT_EQ(
    writeUntil([](return_type r) {return r == return_type::DEACTIVATE;}), return_type::DEACTIVATE);
}

// Another client takes the control from mission (forced by the page or joystick, park on
// overheat): write() returns DEACTIVATE.
TEST_F(BracoSystemTest, WriteAbortsTheGoalWhenTheControlIsTaken)
{
  JointStatesPublisher joint_states(jointStates(6));
  ArmDriverTopics topics;
  startSending(topics);

  topics.owner("joystick");
  EXPECT_EQ(
    writeUntil([](return_type r) {return r == return_type::DEACTIVATE;}), return_type::DEACTIVATE);
}

// mission releasing the control (owner free, no STOP) is the end of a goal, not an error:
// write() stops sending and returns OK.
TEST_F(BracoSystemTest, WriteStopsSendingWhenMissionReleasesTheControl)
{
  JointStatesPublisher joint_states(jointStates(6));
  ArmDriverTopics topics;
  startSending(topics);

  topics.owner("");
  std::this_thread::sleep_for(300ms);   // the free owner arrives
  const auto sent = settledCount(topics);
  EXPECT_TRUE(writeFor(500ms));
  EXPECT_EQ(settledCount(topics), sent);
}

// After an abort and the STOP's release, arm_controller claiming our commands again (the next
// goal) starts clean: write() sends again.
TEST_F(BracoSystemTest, WriteSendsAgainAtTheNextGoalAfterAStop)
{
  JointStatesPublisher joint_states(jointStates(6));
  ArmDriverTopics topics;
  startSending(topics);

  topics.stopped(true);
  ASSERT_EQ(
    writeUntil([](return_type r) {return r == return_type::DEACTIVATE;}), return_type::DEACTIVATE);
  topics.stopped(false);   // released (make resume); mission takes the control again
  release();
  claim();
  const auto before = topics.commands().size();
  writeUntil([&](return_type) {return topics.commands().size() > before;});
  EXPECT_GT(topics.commands().size(), before);
}
