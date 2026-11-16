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

// move_to_pose - the mission node (issues #84, #85). The node is
// braco_mission::MoveToPoseNode (include/braco_mission/move_to_pose_node.hpp).

#include <cstdio>
#include <exception>
#include <memory>
#include <thread>

#include "braco_mission/move_to_pose_node.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{

// The node's whole life. Its constructor and rclcpp may throw (a parameter of the wrong
// type, a failed init): main catches that, so nothing escapes it.
int run(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<braco_mission::MoveToPoseNode>();
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() {executor.spin();});
  int status = 0;
  try {
    node->start();
  } catch (const std::exception & e) {
    RCLCPP_FATAL(node->get_logger(), "%s", e.what());
    status = 1;
    rclcpp::shutdown();
  }
  spinner.join();
  node->stop();
  rclcpp::shutdown();
  return status;
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    return run(argc, argv);
  } catch (const std::exception & e) {
    std::fprintf(stderr, "move_to_pose: %s\n", e.what());
  } catch (...) {
    std::fprintf(stderr, "move_to_pose: unknown exception\n");
  }
  return 1;
}
