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
Run the same list of moves in Gazebo, then (with --real) on the real arm.

Used by park.py and move_arm_example.py. A move is (six angles in degrees, seconds in Gazebo).

  Gazebo:    the arm_controller follow_joint_trajectory action (gazebo.launch.py)
  Real arm:  arm_driver on the Jetson, like braco_bringup's ArmMoves (zero, park): wait for
             the driver, leave rest mode, take the control as "script" (forced, ADR-0002: the
             driver only obeys the owner; given back at the end), power on, then send targets
             at most STEP_DEG ahead of the real angles every RESEND_S, so the driver's 300 ms
             watchdog stops the arm if this program dies; stop on timeout. The driver's
             table-clearance check still applies to every target.

Gazebo and the real arm both publish /joint_states, so they must be on different ROS domains:
Gazebo on the shell's ROS_DOMAIN_ID (e.g. 77), the real arm on --real-domain (default 42).
"""

import argparse
import math
import os
import sys
import time

from braco_interfaces.msg import ArmCommand
from braco_interfaces.srv import TakeControl
from control_msgs.action import FollowJointTrajectory
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

# Real arm, the same values as braco_bringup's arm_moves.cpp, zero.cpp and park.cpp
STEP_DEG = 15.0         # max lead of a target over the real angles
SOURCE = 'script'       # ArmCommand.source and the control owner name, like arm_moves.cpp
RESEND_S = 0.1          # well below the driver's 300 ms watchdog
STATE_TIMEOUT_S = 0.5   # older /joint_states = no feedback
LOOP_S = 0.02
TOLERANCE_DEG = 2.0
DEADBAND_DEG = 3.0     # stopped this close to the target: the servo's dead zone, count it as there
MOVE_TIMEOUT_S = 120.0  # safety cap for one move; slow speeds and long moves take a while
STALL_S = 5.0          # stop if the arm got no closer for this long (refused, blocked, ...)
STALL_PROGRESS_DEG = 0.5
DRIVER_TIMEOUT_S = 10.0
CALL_TIMEOUT_S = 5.0
SETTLE_TIMEOUT_S = 5.0  # Gazebo: after the trajectory, time for the arm to catch up


class _Arm:
    """A node in its own ROS context, so Gazebo and the real arm can be on different domains."""

    def __init__(self, name, domain):
        self.domain = domain
        self.context = rclpy.Context()
        rclpy.init(context=self.context, domain_id=domain)
        self.node = Node(name, context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.log = self.node.get_logger()
        self.angles = None          # degrees, J1..J6
        self.angles_time = 0.0
        self.node.create_subscription(JointState, 'joint_states', self._on_joints, 10)

    def _on_joints(self, msg):
        named = dict(zip(msg.name, msg.position))
        if all(j in named for j in JOINTS):
            self.angles = [math.degrees(named[j]) for j in JOINTS]
            self.angles_time = time.monotonic()

    def fresh_angles(self):
        if self.angles is None or time.monotonic() - self.angles_time > STATE_TIMEOUT_S:
            return None
        return self.angles

    def spin(self, seconds):
        """Handle at most one callback, waiting up to `seconds` for it."""
        self.executor.spin_once(timeout_sec=seconds)

    def pause(self, seconds):
        """Wait `seconds` while still handling callbacks (joint states)."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.spin(min(0.05, max(0.0, end - time.monotonic())))

    def wait(self, future, timeout):
        self.executor.spin_until_future_complete(future, timeout_sec=timeout)
        return future.done()

    def close(self):
        self.executor.shutdown()
        self.node.destroy_node()
        rclpy.shutdown(context=self.context)


class SimArm(_Arm):
    """The Gazebo arm: one trajectory goal per move, waited for."""

    def __init__(self, name, domain):
        super().__init__(name, domain)
        self.client = ActionClient(self.node, FollowJointTrajectory,
                                   '/arm_controller/follow_joint_trajectory')

    def prepare(self):
        if self.client.wait_for_server(timeout_sec=10.0):
            return True
        self.log.error('arm_controller not found on ROS domain %d: is gazebo.launch.py running '
                       '(with the same ROS_DOMAIN_ID)?' % self.domain)
        return False

    def move(self, degrees, seconds):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = [math.radians(d) for d in degrees]
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds % 1) * 1e9)
        goal.trajectory.points = [point]
        handle = self.client.send_goal_async(goal)
        if not self.wait(handle, CALL_TIMEOUT_S) or not handle.result().accepted:
            return False
        result = handle.result().get_result_async()
        if not self.wait(result, seconds + 10.0):
            return False
        if result.result().result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
            return False
        # The controller reports success when the trajectory's time is over, even if the arm
        # lags behind (joint velocity limits): wait until it is really there, like on the arm.
        end = time.monotonic() + SETTLE_TIMEOUT_S
        while time.monotonic() < end:
            self.spin(LOOP_S)
            current = self.fresh_angles()
            if current and max(abs(c - t) for c, t in zip(current, degrees)) <= TOLERANCE_DEG:
                return True
        self.log.error('the arm did not reach the pose within %.0f s after the trajectory: '
                       'too fast for the joint velocity limits? give the move more seconds'
                       % SETTLE_TIMEOUT_S)
        return False

    def stop(self):
        pass


class RealArm(_Arm):
    """The real arm through arm_driver (/arm/command and its arm/* services)."""

    def __init__(self, name, domain, speed):
        super().__init__(name, domain)
        self.speed = speed
        self.command_pub = self.node.create_publisher(ArmCommand, 'arm/command', 10)
        self.power_client = self.node.create_client(Trigger, 'arm/power_on')
        self.rest_client = self.node.create_client(SetBool, 'arm/set_read_only')
        self.release_client = self.node.create_client(Trigger, 'arm/release_servos')
        self.control_client = self.node.create_client(TakeControl, 'arm/take_control')
        self.has_control = False

    def _call(self, client, request, what):
        if not client.wait_for_service(timeout_sec=CALL_TIMEOUT_S):
            self.log.error('%s failed: arm_driver has no %s service' % (what, client.srv_name))
            return False
        future = client.call_async(request)
        if not self.wait(future, CALL_TIMEOUT_S):
            self.log.error('%s failed: timeout' % what)
            return False
        if not future.result().success:
            self.log.error('%s failed: %s' % (what, future.result().message))
            return False
        return True

    def set_rest(self, on):
        """Rest mode: the driver refuses motion and power on, temperatures keep coming."""
        request = SetBool.Request()
        request.data = on
        return self._call(self.rest_client, request, 'rest mode %s' % ('on' if on else 'off'))

    def release_servos(self):
        """Free all joints: the arm goes limp, so only from a resting pose (park)."""
        return self._call(self.release_client, Trigger.Request(), 'release servos')

    def take_control(self):
        """arm_driver only obeys the control owner: motion from any other source is ignored."""
        request = TakeControl.Request()
        request.source = SOURCE
        request.force = True
        self.has_control = self._call(self.control_client, request, 'take control')
        return self.has_control

    def release_control(self):
        if not self.has_control:
            return
        self.has_control = False
        request = TakeControl.Request()
        request.source = SOURCE
        request.release = True
        self._call(self.control_client, request, 'release control')

    def prepare(self):
        end = time.monotonic() + DRIVER_TIMEOUT_S
        while time.monotonic() < end:
            self.spin(0.1)
            if self.fresh_angles() is not None and self.power_client.service_is_ready():
                break
        else:
            self.log.error('arm_driver not found on ROS domain %d: is the arm up on the Jetson '
                           '(make up)?' % self.domain)
            return False
        self.set_rest(False)   # leave rest mode (set by park), or power on is refused
        return self.take_control() and self._call(self.power_client, Trigger.Request(), 'power on')

    def move(self, degrees, seconds=None):
        """
        Go to `degrees`; `seconds` is for Gazebo only, the real arm moves at self.speed.

        Stops the arm and logs why if it gets no closer for STALL_S, loses its joint states,
        or takes longer than MOVE_TIMEOUT_S.
        """
        start = time.monotonic()
        end = start + MOVE_TIMEOUT_S
        next_send = 0.0
        best, best_time = None, start
        reason = 'took longer than %.0f s' % MOVE_TIMEOUT_S
        while time.monotonic() < end and rclpy.ok(context=self.context):
            self.spin(LOOP_S)
            now = time.monotonic()
            current = self.fresh_angles()
            if current is None:
                if now - max(self.angles_time, start) > STATE_TIMEOUT_S * 4:
                    reason = 'no /joint_states from arm_driver for %.0f s' % (STATE_TIMEOUT_S * 4)
                    break
                continue
            deviation = max(abs(c - t) for c, t in zip(current, degrees))
            if deviation <= TOLERANCE_DEG:
                return True
            if best is None or deviation < best - STALL_PROGRESS_DEG:
                best, best_time = deviation, now
            elif now - best_time > STALL_S:
                if deviation <= DEADBAND_DEG:
                    self.log.warning('real arm stopped %.1f deg from the target (servo dead '
                                     'zone): close enough' % deviation)
                    return True
                reason = ('no progress for %.0f s (move refused by the driver, e.g. the table '
                          'check, or a joint blocked?)' % STALL_S)
                break
            if now < next_send:
                continue
            next_send = now + RESEND_S
            # Same fraction for every joint, so they all arrive together
            scale = min(1.0, STEP_DEG / deviation)
            command = ArmCommand()
            command.type = ArmCommand.ALL_ANGLES
            command.angles_deg = [round(c + (t - c) * scale, 1) for c, t in zip(current, degrees)]
            command.speed = self.speed
            command.source = SOURCE
            self.command_pub.publish(command)
        self.stop()
        now_at = [round(a, 1) for a in self.angles] if self.angles else None
        self.log.error('real arm stopped: %s. Target %s, arm at %s'
                       % (reason, [round(d, 1) for d in degrees], now_at))
        return False

    def stop(self):
        command = ArmCommand()
        command.type = ArmCommand.STOP
        self.command_pub.publish(command)


def _run_on(arm, label, moves):
    if not arm.prepare():
        return False
    for degrees, seconds in moves:
        if not arm.move(degrees, seconds):
            arm.log.error('%s: move to %s FAILED' % (label, degrees))
            return False
        arm.pause(0.3)
        now = [round(a, 1) for a in arm.angles] if arm.angles else None
        arm.log.info('%s: reached %s (arm at %s)' % (label, degrees, now))
    return True


def run(name, moves, description='', after_real=None, after_real_note=''):
    """
    Run `moves` [(degrees J1..J6, seconds), ...] in Gazebo, then with --real on the arm.

    after_real(arm) -> bool runs on the real arm once all moves are done (e.g. park releases
    the joints); after_real_note says what it does in the confirmation question.
    """
    parser = argparse.ArgumentParser(prog=name, description=description)
    parser.add_argument('--real', action='store_true',
                        help='after Gazebo, run the same moves on the real arm (asks first)')
    parser.add_argument('--yes', action='store_true',
                        help='with --real: do not ask before moving the real arm')
    parser.add_argument('--skip-sim', action='store_true',
                        help='with --real: go straight to the real arm')
    parser.add_argument('--sim-domain', type=int,
                        default=int(os.environ.get('ROS_DOMAIN_ID', '0')),
                        help="Gazebo's ROS domain (default: this shell's ROS_DOMAIN_ID)")
    parser.add_argument('--real-domain', type=int, default=42,
                        help="the real arm's ROS domain (default 42, as in docker/.env)")
    parser.add_argument('--speed', type=int, default=30,
                        help='real arm speed, 1..100 (default 30, like zero and park)')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])

    if args.skip_sim and not args.real:
        parser.error('--skip-sim needs --real')
    if args.real and not args.skip_sim and args.sim_domain == args.real_domain:
        parser.error('Gazebo and the real arm are both on ROS domain %d: their /joint_states '
                     'would mix. Run Gazebo and this script with another ROS_DOMAIN_ID '
                     '(e.g. export ROS_DOMAIN_ID=77).' % args.sim_domain)
    if not 1 <= args.speed <= 100:
        parser.error('--speed must be 1..100')

    if not args.skip_sim:
        sim = SimArm(name + '_sim', args.sim_domain)
        try:
            ok = _run_on(sim, 'gazebo', moves)
        finally:
            sim.close()
        if not ok:
            return 1
    if not args.real:
        return 0

    if not args.yes:
        then = ', then %s' % after_real_note if after_real_note else ''
        answer = input('Run the same moves on the REAL arm (ROS domain %d, speed %d%s)? '
                       'Keep the emergency stop at hand. [y/N] '
                       % (args.real_domain, args.speed, then))
        if answer.strip().lower() not in ('y', 'yes'):
            print('real arm not moved')
            return 0
    real = RealArm(name + '_real', args.real_domain, args.speed)
    try:
        ok = _run_on(real, 'real arm', moves)
        if ok and after_real is not None:
            ok = after_real(real)
    except KeyboardInterrupt:
        real.stop()
        real.log.warning('interrupted: arm stopped')
        ok = False
    finally:
        real.release_control()
        real.close()
    return 0 if ok else 1
