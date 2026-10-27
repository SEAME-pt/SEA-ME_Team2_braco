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
Move the arm through a few poses: in Gazebo, then with --real on the real arm.

An example to copy: write the moves once, try them in Gazebo, then run them on the arm.

  ros2 launch braco_description gazebo.launch.py          # Gazebo (ROS_DOMAIN_ID=77)
  ros2 run braco_description move_arm_example.py          # Gazebo only
  ros2 run braco_description move_arm_example.py --real   # Gazebo, then the real arm (asks first)
"""

import sys

from braco_arm import run

# (J1..J6 in degrees, seconds in Gazebo; the real arm moves at --speed)
MOVES = [
    ([30, 0, 0, 0, 0, 0], 2.0),
    ([30, -20, 40, 0, 0, 0], 2.0),
    ([0, 0, 0, 0, 0, 0], 2.0),
]


def main():
    return run('move_arm_example', MOVES, __doc__)


if __name__ == '__main__':
    sys.exit(main())
