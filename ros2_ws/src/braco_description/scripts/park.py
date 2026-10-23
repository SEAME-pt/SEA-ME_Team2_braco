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
Park the arm: in Gazebo, then with --real on the real arm.

  ros2 launch braco_description gazebo.launch.py     # Gazebo (ROS_DOMAIN_ID=77)
  ros2 run braco_description park.py                 # Gazebo only
  ros2 run braco_description park.py --real          # Gazebo, then the real arm (asks first)

On the real arm it does what `make park` does (braco_bringup park.cpp): fold into the park
pose, release the joints and turn rest mode on. If the pose is not reached the joints stay
locked. `make resume` or `make zero` leaves rest mode. Gazebo has no joints to release.
"""

import sys

from braco_arm import run

# PARK_ANGLES of ros2_ws/src/mycobot_cpp/includes/mycobot.hpp, J1..J6 in degrees
# (test_urdf.py keeps them equal)
PARK_DEG = [0, 131, -148, 146, 4.5, 0]
SECONDS = 4.0   # in Gazebo; the real arm moves at --speed


def release(arm):
    """Real arm only, once in the park pose: free the joints, then rest mode on."""
    arm.pause(0.5)   # let the arm settle
    if not arm.release_servos():
        return False
    arm.set_rest(True)   # best effort, like park.cpp: the joints are already free
    arm.log.info('real arm: parked - joints released, rest mode on')
    return True


def main():
    return run('park', [(PARK_DEG, SECONDS)], __doc__,
               after_real=release, after_real_note='release the joints and rest')


if __name__ == '__main__':
    sys.exit(main())
