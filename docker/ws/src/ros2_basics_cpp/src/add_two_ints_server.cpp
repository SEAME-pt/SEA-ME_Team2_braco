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

// Service server: returns a + b.
#include <cinttypes>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "seame_interfaces/srv/add_two_ints.hpp"

using AddTwoInts = seame_interfaces::srv::AddTwoInts;

class AddTwoIntsServer : public rclcpp::Node
{
public:
  AddTwoIntsServer()
  : Node("add_two_ints_server_cpp")
  {
    srv_ = create_service<AddTwoInts>(
      "add_two_ints",
      [this](const std::shared_ptr<AddTwoInts::Request> request,
      std::shared_ptr<AddTwoInts::Response> response) {
        response->sum = request->a + request->b;
        RCLCPP_INFO(
          get_logger(), "%" PRId64 " + %" PRId64 " = %" PRId64,
          request->a, request->b, response->sum);
      });
  }

private:
  rclcpp::Service<AddTwoInts>::SharedPtr srv_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AddTwoIntsServer>());
  rclcpp::shutdown();
  return 0;
}
