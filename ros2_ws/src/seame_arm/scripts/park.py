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
Shutdown routine: fold the arm into the park pose, then release all joints.

ROS 2 version of ~/scripts/park.py: goes through arm_driver instead of
opening the serial port. Run by arm_routine.sh before the container stops.
"""

import sys

from arm_moves import run

SPEED = 30
PARK_ANGLES = [0, 131, -148, 146, 4.5, 0]   # J3 -148: just inside the -150 firmware limit
TOLERANCE_DEG = 2.0
TIMEOUT_S = 20
DRIVER_TIMEOUT_S = 10


def park(arm):
    if not arm.wait_for_driver(DRIVER_TIMEOUT_S):
        return False
    arm.set_rest(False)   # rest mode refuses the park move itself
    # The park pose is outside the driver's soft limits (J2, J3, J4): use the firmware ones
    if not arm.move_to(PARK_ANGLES, SPEED, TOLERANCE_DEG, TIMEOUT_S, firmware_limits=True):
        arm.get_logger().error('Park position not reached - joints left locked')
        return False
    arm.wait(0.5)   # let the arm settle
    if not arm.release_servos():
        return False
    # The driver stays up, only watching temperatures, until `make arm-resume`
    arm.set_rest(True)
    arm.get_logger().info('Parked - joints released, rest mode on')
    return True


if __name__ == '__main__':
    sys.exit(run('arm_park', park))
