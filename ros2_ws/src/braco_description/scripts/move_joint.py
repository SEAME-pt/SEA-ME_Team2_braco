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
Move one joint of the simulated arm to an angle; the other five stay where they are.

  ros2 run braco_description move_joint.py 2 30              # joint2 to +30 deg, in 2 s
  ros2 run braco_description move_joint.py 1 -45 --seconds 4

Sends one goal to /arm_controller/follow_joint_trajectory (the controllers of
config/controllers.yaml) and waits until /joint_states shows the joint there. Gazebo must be
running on this shell's ROS_DOMAIN_ID (a dev shell and the `sim` container: 77). Simulation
only: the real arm is moved by the page, the joystick, make zero / park, or braco_arm.py's
--real.
"""

import argparse
import os
import sys

from braco_arm import JOINTS, SimArm, STATE_TIMEOUT_S
from rclpy.utilities import remove_ros_args

# JOINT_LIMITS of ros2_ws/src/mycobot_cpp/includes/mycobot.hpp, J1..J6 in degrees
# (test_urdf.py keeps them equal)
LIMITS_DEG = [(-166, 166), (-138, 138), (-149, 149), (-149, 149), (-153, 158), (-178, 178)]


def main():
    parser = argparse.ArgumentParser(prog='move_joint', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('joint', type=int, choices=range(1, 7), metavar='JOINT',
                        help='joint number, 1..6')
    parser.add_argument('degrees', type=float, help='target angle in degrees')
    parser.add_argument('--seconds', type=float, default=2.0,
                        help='time to reach it (default 2)')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    lo, hi = LIMITS_DEG[args.joint - 1]
    if not lo <= args.degrees <= hi:
        parser.error('joint%d goes from %g to %g degrees' % (args.joint, lo, hi))
    if args.seconds <= 0:
        parser.error('--seconds must be positive')

    arm = SimArm('move_joint', int(os.environ.get('ROS_DOMAIN_ID', '0')))
    try:
        if not arm.prepare():
            return 1
        # The other joints keep their current angles: read them first
        arm.pause(2 * STATE_TIMEOUT_S)
        current = arm.fresh_angles()
        if current is None:
            arm.log.error('no /joint_states from Gazebo on ROS domain %d' % arm.domain)
            return 1
        target = list(current)
        target[args.joint - 1] = args.degrees
        arm.log.info('%s: %.1f -> %.1f deg in %.1f s' % (
            JOINTS[args.joint - 1], current[args.joint - 1], args.degrees, args.seconds))
        if not arm.move(target, args.seconds):
            arm.log.error('move FAILED')
            return 1
        arm.pause(0.3)   # move() returns within TOLERANCE_DEG: let it settle, like braco_arm.py
        arm.log.info('%s at %.1f deg' % (JOINTS[args.joint - 1],
                                         arm.angles[args.joint - 1]))
        return 0
    finally:
        arm.close()


if __name__ == '__main__':
    sys.exit(main())
