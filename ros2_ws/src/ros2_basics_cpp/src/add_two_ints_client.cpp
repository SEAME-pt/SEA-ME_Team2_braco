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

// Service client: ros2 run ros2_basics_cpp add_two_ints_client 2 3
#include <chrono>
#include <cinttypes>
#include <cstdlib>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "seame_interfaces/srv/add_two_ints.hpp"

using AddTwoInts = seame_interfaces::srv::AddTwoInts;
using namespace std::chrono_literals;

int main(int argc, char ** argv)
{
  const std::vector<std::string> args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  if (args.size() != 3) {
    std::fprintf(stderr, "usage: add_two_ints_client A B\n");
    rclcpp::shutdown();
    return 1;
  }

  auto node = rclcpp::Node::make_shared("add_two_ints_client_cpp");
  auto client = node->create_client<AddTwoInts>("add_two_ints");

  auto request = std::make_shared<AddTwoInts::Request>();
  request->a = std::atoll(args[1].c_str());
  request->b = std::atoll(args[2].c_str());

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(node->get_logger(), "interrupted while waiting for the service");
      return 1;
    }
    RCLCPP_INFO(node->get_logger(), "waiting for service /add_two_ints...");
  }

  auto future = client->async_send_request(request);
  if (rclcpp::spin_until_future_complete(node, future) == rclcpp::FutureReturnCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Result: %" PRId64, future.get()->sum);
  } else {
    RCLCPP_ERROR(node->get_logger(), "service call failed");
  }

  rclcpp::shutdown();
  return 0;
}
