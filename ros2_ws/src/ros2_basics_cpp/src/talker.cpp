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

// Publisher with an explicit QoS profile.
#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Talker : public rclcpp::Node
{
public:
  Talker()
  : Node("talker_cpp")
  {
    const double rate_hz = declare_parameter("publish_rate_hz", 1.0);

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10))
      .reliable()
      .durability_volatile();
    pub_ = create_publisher<std_msgs::msg::String>("chatter", qos);

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / rate_hz), [this]() {on_timer();});
  }

private:
  void on_timer()
  {
    std_msgs::msg::String msg;
    msg.data = "Hello from C++: " + std::to_string(count_++);
    RCLCPP_INFO(get_logger(), "Publishing: \"%s\"", msg.data.c_str());
    pub_->publish(msg);
  }

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_{0};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Talker>());
  rclcpp::shutdown();
  return 0;
}
