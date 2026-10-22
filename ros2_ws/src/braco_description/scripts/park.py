#!/usr/bin/env python3
# Copyright 2026 SEA:ME Team 2
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""
Move the arm in Gazebo to the park pose (the folded pose of the real arm).

Start Gazebo first, then in another shell:

  ros2 launch braco_description gazebo.launch.py
  ros2 run braco_description park.py

Simulation only: the real arm parks with `make park` on the Jetson (braco_bringup park).
"""

from move_arm_example import MoveArm  # same folder (src/scripts and install/lib)
import rclpy

# PARK_ANGLES of libs/cpp/includes/mycobot.hpp, J1..J6 in degrees (test_urdf.py keeps them equal)
PARK_DEG = [0, 131, -148, 146, 4.5, 0]
SECONDS = 4.0


def main():
    rclpy.init()
    node = MoveArm('park')
    try:
        ok = node.move(PARK_DEG, SECONDS)
        node.get_logger().info('park %s, arm at %s'
                               % ('ok' if ok else 'FAILED', node.current_degrees()))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
