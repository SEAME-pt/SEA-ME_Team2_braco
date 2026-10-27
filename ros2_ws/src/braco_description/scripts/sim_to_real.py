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
Sim-to-real checks of issue #25 (docs/urdf-sim-to-real.md): Gazebo against the real arm.

Gazebo on this shell's ROS_DOMAIN_ID (e.g. 77), the real arm on --real-domain (default 42).

  directions   step 2: each joint +30 deg in Gazebo, then on the real arm; you answer whether
               they moved the same joint the same way; prints the table for the report
  fk           step 3: reads the real arm's angles, puts Gazebo there and prints the model's
               tool0 position in base_link (mm), to compare with mc.get_coords();
               --pose J1..J6 (deg) first moves Gazebo there, then (asks) the real arm
  velocity     step 4: shows each joint's sweep in Gazebo, then times it on the real arm at
               speeds 25/50/75/100 (middle 80 % of the sweep); prints rad/s per joint and
               speed, and the speed-100 values for the URDF

Every real move asks first; Ctrl-C stops the arm.
"""

import argparse
import math
import os
import subprocess
import sys
import time
import xml.etree.ElementTree as ET

from ament_index_python.packages import get_package_share_directory
from braco_arm import JOINTS, RealArm, SimArm, SOURCE
from braco_interfaces.msg import ArmCommand
from rclpy.utilities import remove_ros_args

TEST_DEG = 30.0
# velocity: half range per joint (from -A to +A), all other joints at 0 (arm upright)
VELOCITY_RANGE_DEG = [90, 45, 60, 60, 60, 90]
VELOCITY_SPEEDS = [25, 50, 75, 100]
VELOCITY_TIMEOUT_S = 30.0
VELOCITY_TOLERANCE_DEG = 2.0


def ask(question):
    return input(question + ' [y/N] ').strip().lower() in ('y', 'yes')


def zero():
    return [0.0] * 6


def directions(sim, real):
    rows = []
    for j in range(6):
        pose = zero()
        pose[j] = TEST_DEG
        print('\n--- joint%d: +%.0f deg. Watch Gazebo.' % (j + 1, TEST_DEG))
        if not sim.move(pose, 2.0):
            return False
        if not ask('Gazebo moved joint%d. Now the real arm, same move?' % (j + 1)):
            break
        if not real.move(pose):
            return False
        same_joint = ask('Did the real arm move the SAME joint?')
        same_way = ask('In the SAME direction as Gazebo?')
        note = input('Note (optional): ').strip()
        rows.append((j + 1, same_joint, same_way, note))
        if not (sim.move(zero(), 2.0) and real.move(zero())):
            return False
    print('\n| Joint | Same joint? | Same direction? | Note |\n|---|---|---|---|')
    for j, a, b, note in rows:
        same_joint = 'yes' if a else '**NO**'
        same_way = 'yes' if b else '**NO**'
        print('| joint%d | %s | %s | %s |' % (j, same_joint, same_way, note))
    return True


def _rpy(r, p, y):
    cr, sr, cp, sp, cy, sy = (math.cos(r), math.sin(r), math.cos(p), math.sin(p),
                              math.cos(y), math.sin(y))
    return [[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
            [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
            [-sp, cp * sr, cp * cr]]


def _mul(a, b):
    rot = [[sum(a[0][i][k] * b[0][k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    return rot, [a[1][i] + sum(a[0][i][k] * b[1][k] for k in range(3)) for i in range(3)]


def tool0_in_base_link(degrees):
    """Forward kinematics of the URDF (what tf2_echo base_link tool0 shows), in metres."""
    model = os.path.join(get_package_share_directory('braco_description'), 'urdf',
                         'braco.urdf.xacro')
    urdf = ET.fromstring(subprocess.check_output(['xacro', model]))
    parent = {j.find('child').get('link'): j for j in urdf.findall('joint')}
    chain, link = [], 'tool0'
    while link != 'base_link':
        joint = parent[link]
        chain.insert(0, joint)
        link = joint.find('parent').get('link')
    pose = ([[1, 0, 0], [0, 1, 0], [0, 0, 1]], [0, 0, 0])
    for joint in chain:
        origin = joint.find('origin')
        xyz = [float(v) for v in origin.get('xyz', '0 0 0').split()]
        pose = _mul(pose, (_rpy(*[float(v) for v in origin.get('rpy', '0 0 0').split()]), xyz))
        if joint.get('name') in JOINTS:
            q = math.radians(degrees[JOINTS.index(joint.get('name'))])
            spin = [[math.cos(q), -math.sin(q), 0], [math.sin(q), math.cos(q), 0], [0, 0, 1]]
            pose = _mul(pose, (spin, [0, 0, 0]))
    return pose[1]


def fk(sim, real):
    real.pause(0.5)
    angles = real.fresh_angles()
    if angles is None:
        real.log.error('no fresh /joint_states from the real arm')
        return False
    print('real arm angles (deg): %s' % [round(a, 1) for a in angles])
    print("putting Gazebo at the real arm's actual angles ...")
    sim.move(angles, 3.0)
    xyz = tool0_in_base_link(angles)
    print('model tool0 in base_link (mm): X %.1f  Y %.1f  Z %.1f' % tuple(v * 1000 for v in xyz))
    print('Now read the real arm: make down on the Jetson, then mc.get_coords() (X, Y, Z in mm),'
          ' then make up. Put both in the FK table.')
    return True


def _time_move(real, joint, target, speed):
    """
    Send `target` (all six angles) at `speed`, re-sent so the watchdog stays fed.

    Only `joint` counts (the others rest about 1 deg off their targets: servo dead zone).
    Return (rad/s over the middle 80 % of the sweep, peak rad/s, where it stopped), or
    (None, peak, angle) if it did not get through.
    """
    start = real.fresh_angles()[joint]
    sweep = abs(target[joint] - start)
    t10 = t90 = None
    peak, last = 0.0, None
    still_since = time.monotonic()
    end = time.monotonic() + VELOCITY_TIMEOUT_S
    next_send = 0.0
    angle = start
    while time.monotonic() < end:
        real.spin(0.01)
        now = time.monotonic()
        if now >= next_send:
            next_send = now + 0.1
            command = ArmCommand()
            command.type = ArmCommand.ALL_ANGLES
            command.angles_deg = [float(v) for v in target]
            command.speed = speed
            command.source = SOURCE
            real.command_pub.publish(command)
        current = real.fresh_angles()
        if current is None or (last is not None and real.angles_time <= last[1]):
            continue
        angle, stamp = current[joint], real.angles_time
        if last is not None:
            peak = max(peak, abs(angle - last[0]) / (stamp - last[1]))
            if abs(angle - last[0]) > 0.2:
                still_since = now
        last = (angle, stamp)
        done = abs(angle - start) / sweep
        if t10 is None and done >= 0.1:
            t10 = stamp
        if t90 is None and done >= 0.9:
            t90 = stamp
        arrived = abs(angle - target[joint]) <= VELOCITY_TOLERANCE_DEG
        if t90 is not None and (arrived or now - still_since > 1.0):
            return math.radians(0.8 * sweep) / (t90 - t10), math.radians(peak), angle
    real.stop()
    if t90 is not None:   # got through the timed part, only the end is short of the target
        return math.radians(0.8 * sweep) / (t90 - t10), math.radians(peak), angle
    return None, math.radians(peak), angle


def velocity(sim, real):
    results = {}
    for j in range(6):
        a = VELOCITY_RANGE_DEG[j]
        lo, hi = zero(), zero()
        lo[j], hi[j] = -a, a
        sweep_s = max(3.0, math.radians(2 * a) / 0.8)   # within Gazebo's 1 rad/s limit
        print('\n--- joint%d: from %+d to %+d deg. Watch Gazebo.' % (j + 1, -a, a))
        preview = [(lo, sweep_s / 2), (hi, sweep_s), (zero(), sweep_s / 2)]
        if not all(sim.move(pose, seconds) for pose, seconds in preview):
            return False
        if not ask('Same sweep on the REAL arm at speeds %s (others at 0)?' % VELOCITY_SPEEDS):
            continue
        for speed in VELOCITY_SPEEDS:
            if not real.move(lo):
                return False
            real.pause(0.5)
            if not real.take_control():   # _time_move sends its own commands, not move()
                return False
            avg, peak, stopped = _time_move(real, j, hi, speed)
            if avg is None:
                real.log.error('joint%d speed %d: did not get through the sweep (stopped at %.1f '
                               'deg, target %+d)' % (j + 1, speed, stopped, a))
                return False
            results[(j, speed)] = (avg, peak)
            print('joint%d speed %3d: %.2f rad/s (%.0f deg/s) over the middle of the sweep, '
                  'peak %.2f rad/s' % (j + 1, speed, avg, math.degrees(avg), peak))
        if not real.move(zero()):
            return False
    print('\nrad/s over the middle 80 %% of the sweep (peak):\n')
    print('| Joint | ' + ' | '.join('speed %d' % sp for sp in VELOCITY_SPEEDS) + ' |')
    print('|---|' + '---|' * len(VELOCITY_SPEEDS))
    for j in range(6):
        cells = ['%.2f (%.2f)' % results[(j, sp)] if (j, sp) in results else '-'
                 for sp in VELOCITY_SPEEDS]
        print('| joint%d | %s |' % (j + 1, ' | '.join(cells)))
    top = ['joint%d %.2f' % (j + 1, results[(j, 100)][0]) for j in range(6) if (j, 100) in results]
    if top:
        print('\nURDF velocity (speed 100, middle of the sweep), rad/s: %s' % ', '.join(top))
    return True


def main():
    parser = argparse.ArgumentParser(prog='sim_to_real',
                                     description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('check', choices=['directions', 'fk', 'velocity'])
    parser.add_argument('--sim-domain', type=int,
                        default=int(os.environ.get('ROS_DOMAIN_ID', '0')))
    parser.add_argument('--real-domain', type=int, default=42)
    parser.add_argument('--speed', type=int, default=15,
                        help='real arm speed for directions and fk positioning (default 15)')
    parser.add_argument('--pose', type=float, nargs=6, metavar='DEG',
                        help='fk: first move Gazebo, then (asks) the real arm to these angles')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if args.sim_domain == args.real_domain:
        parser.error('Gazebo and the real arm are both on ROS domain %d: export another '
                     'ROS_DOMAIN_ID for Gazebo and this script (e.g. 77)' % args.sim_domain)

    sim = SimArm('sim_to_real_sim', args.sim_domain)
    real = RealArm('sim_to_real_real', args.real_domain, args.speed)
    try:
        if not sim.prepare():
            return 1
        if args.check == 'fk':
            if args.pose:
                print('Gazebo first (preview) ...')
                if not sim.move(args.pose, 3.0):
                    return 1
                if not ask('Gazebo is at %s. Move the REAL arm (ROS domain %d) there at speed %d?'
                           ' Emergency stop at hand?' % (args.pose, args.real_domain, args.speed)):
                    return 0
                if not (real.prepare() and real.move(args.pose)):
                    return 1
            else:   # only read the real arm: no power on, no move
                end = time.monotonic() + 10.0
                while real.fresh_angles() is None and time.monotonic() < end:
                    real.spin(0.1)
            return 0 if fk(sim, real) else 1
        if not ask('This moves the REAL arm (ROS domain %d). Emergency stop at hand?'
                   % args.real_domain):
            return 0
        if not real.prepare():
            return 1
        check = directions if args.check == 'directions' else velocity
        return 0 if check(sim, real) else 1
    except KeyboardInterrupt:
        real.stop()
        print('\ninterrupted: real arm stopped')
        return 1
    finally:
        real.release_control()
        sim.close()
        real.close()


if __name__ == '__main__':
    sys.exit(main())
