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
Move the arm to a pose through arm_driver (used by zero.py and park.py).

The scripts cannot open /dev/ttyTHS1 like the old pymycobot versions did:
arm_driver owns the port. Instead they:
  * wait for arm_driver (its /joint_states and services),
  * send bounded steps toward the pose (at most STEP_DEG ahead of the real
    angles, all joints arriving together), resent every RESEND_S, so the
    driver's watchdog stops the arm if the script dies half-way,
  * check the real angles until the pose is reached or the timeout expires.
"""

import math
import time

from braco_interfaces.msg import ArmCommand
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool, Trigger

STEP_DEG = 15.0         # max lead of a target over the real angles
RESEND_S = 0.1          # well below the driver's 300 ms watchdog
STATE_TIMEOUT_S = 0.5   # older /joint_states = no feedback


class ArmMoves(Node):
    """Small client of arm_driver: pose moves and the Trigger services."""

    def __init__(self, name):
        super().__init__(name)
        self._angles = None
        self._angles_time = 0.0
        self.create_subscription(JointState, 'joint_states', self._on_joints, 10)
        self._cmd = self.create_publisher(ArmCommand, 'arm/command', 10)
        self._power = self.create_client(Trigger, 'arm/power_on')
        self._release = self.create_client(Trigger, 'arm/release_servos')
        self._rest = self.create_client(SetBool, 'arm/set_read_only')

    def _on_joints(self, msg):
        if len(msg.position) < 6:
            return
        self._angles = [math.degrees(p) for p in msg.position[:6]]
        self._angles_time = time.monotonic()

    def angles(self):
        """Real joint angles in degrees, or None if they are not fresh."""
        if self._angles is None or time.monotonic() - self._angles_time > STATE_TIMEOUT_S:
            return None
        return self._angles

    def wait_for_driver(self, timeout_s):
        """Wait until arm_driver publishes angles and offers its services."""
        end = time.monotonic() + timeout_s
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if (self.angles() is not None and self._power.service_is_ready() and
                    self._release.service_is_ready()):
                return True
        self.get_logger().error('arm_driver not available after %.0f s' % timeout_s)
        return False

    def _call(self, client, what):
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None or not result.success:
            why = result.message if result else 'timeout'
            self.get_logger().error('%s failed: %s' % (what, why))
            return False
        return True

    def power_on(self):
        return self._call(self._power, 'power on')

    def release_servos(self):
        return self._call(self._release, 'release servos')

    def set_rest(self, on):
        """
        Rest mode: the driver keeps publishing temperatures but refuses motion.

        Best effort (an older driver has no such service): a failure only logs.
        """
        if not self._rest.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning('arm/set_read_only not available - rest mode not set')
            return False
        future = self._rest.call_async(SetBool.Request(data=on))
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        result = future.result()
        if result is None or not result.success:
            reason = result.message if result else 'timeout'
            self.get_logger().warning('rest mode not set: %s' % reason)
            return False
        return True

    def stop(self):
        self._cmd.publish(ArmCommand(type=ArmCommand.STOP))

    def wait(self, seconds):
        """Sleep while still processing callbacks."""
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def move_to(self, pose, speed, tolerance_deg, timeout_s, firmware_limits=False):
        """
        Move all joints to `pose` (degrees). True once every joint is within tolerance.

        firmware_limits: allow a pose outside the driver's soft limits (the park pose).
        On timeout or lost feedback the arm is stopped and False returned.
        """
        end = time.monotonic() + timeout_s
        next_send = 0.0
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            angles = self.angles()
            if angles is None:
                continue
            deviation = max(abs(a - p) for a, p in zip(angles, pose))
            if deviation <= tolerance_deg:
                return True
            now = time.monotonic()
            if now < next_send:
                continue
            next_send = now + RESEND_S
            # Same fraction for every joint, so they all arrive together
            scale = min(1.0, STEP_DEG / deviation)
            step = [round(a + (p - a) * scale, 1) for a, p in zip(angles, pose)]
            self._cmd.publish(ArmCommand(
                type=ArmCommand.ALL_ANGLES, angles_deg=step, speed=speed,
                use_firmware_limits=firmware_limits))
        self.stop()
        return False


def run(name, action):
    """Init ROS, run action(node) -> bool, shut down; return a process exit code."""
    rclpy.init()
    node = ArmMoves(name)
    try:
        ok = action(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        ok = False   # interrupted: arm_driver's watchdog stops the arm
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0 if ok else 1
