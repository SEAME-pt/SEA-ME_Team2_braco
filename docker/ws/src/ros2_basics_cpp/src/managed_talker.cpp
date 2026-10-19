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

// Lifecycle (managed) talker: publishes on lifecycle_chatter only while Active.
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class ManagedTalker : public rclcpp_lifecycle::LifecycleNode
{
public:
  ManagedTalker()
  : LifecycleNode("managed_talker_cpp")
  {
    RCLCPP_INFO(get_logger(), "created (Unconfigured)");
  }

  CallbackReturn on_configure(const rclcpp_lifecycle::State &) override
  {
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();
    pub_ = create_publisher<std_msgs::msg::String>("lifecycle_chatter", qos);
    timer_ = create_wall_timer(1s, [this]() {on_timer();});
    RCLCPP_INFO(get_logger(), "on_configure: publisher and timer created");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_activate(const rclcpp_lifecycle::State &) override
  {
    pub_->on_activate();
    RCLCPP_INFO(get_logger(), "on_activate: publishing");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override
  {
    pub_->on_deactivate();
    RCLCPP_INFO(get_logger(), "on_deactivate: paused");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_cleanup(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    pub_.reset();
    RCLCPP_INFO(get_logger(), "on_cleanup: resources released");
    return CallbackReturn::SUCCESS;
  }

  CallbackReturn on_shutdown(const rclcpp_lifecycle::State &) override
  {
    timer_.reset();
    pub_.reset();
    RCLCPP_INFO(get_logger(), "on_shutdown");
    return CallbackReturn::SUCCESS;
  }

private:
  void on_timer()
  {
    if (!pub_ || !pub_->is_activated()) {
      return;
    }
    std_msgs::msg::String msg;
    msg.data = "Lifecycle C++: " + std::to_string(count_++);
    RCLCPP_INFO(get_logger(), "Publishing: \"%s\"", msg.data.c_str());
    pub_->publish(msg);
  }

  rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ManagedTalker>();
  // A lifecycle node is not an rclcpp::Node: spin its base interface.
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
