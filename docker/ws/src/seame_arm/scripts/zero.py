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
Boot routine: send the arm to the zero pose (all joints at 0 degrees).

ROS 2 version of ~/scripts/zero.py: goes through arm_driver instead of
opening the serial port. Run by arm_routine.sh when the container starts.
"""

import sys

from arm_moves import run

SPEED = 30
TARGET_ANGLES = [0, 0, 0, 0, 0, 0]
TOLERANCE_DEG = 2.0
TIMEOUT_S = 20
DRIVER_TIMEOUT_S = 60   # at boot arm_driver may still be starting


def zero(arm):
    if not arm.wait_for_driver(DRIVER_TIMEOUT_S):
        return False
    arm.power_on()   # the joints may still be free from the last park
    if arm.move_to(TARGET_ANGLES, SPEED, TOLERANCE_DEG, TIMEOUT_S):
        arm.get_logger().info('At zero')
        return True
    arm.get_logger().error('Zero position not reached - arm stopped')
    return False


if __name__ == '__main__':
    sys.exit(run('arm_zero', zero))
