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
Move the arm to the park pose: in Gazebo, then with --real on the real arm.

  ros2 launch braco_description gazebo.launch.py     # Gazebo (ROS_DOMAIN_ID=77)
  ros2 run braco_description park.py                 # Gazebo only
  ros2 run braco_description park.py --real          # Gazebo, then the real arm (asks first)

Only the move: `make park` on the Jetson also releases the joints and enters rest mode.
"""

import sys

from braco_arm import run

# PARK_ANGLES of libs/cpp/includes/mycobot.hpp, J1..J6 in degrees (test_urdf.py keeps them equal)
PARK_DEG = [0, 131, -148, 146, 4.5, 0]
SECONDS = 4.0   # in Gazebo; the real arm moves at --speed


def main():
    return run('park', [(PARK_DEG, SECONDS)], __doc__)


if __name__ == '__main__':
    sys.exit(main())
