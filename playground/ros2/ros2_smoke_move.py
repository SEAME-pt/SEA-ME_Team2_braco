#!/usr/bin/env python3
"""ROS2 smoke test for the myCobot 280 Jetson Nano (docs/ros2.md, Phase 4).

Sends a small, slow, reversible motion to the arm through the ROS2 topic
bridge and verifies that telemetry comes back.

Plan (all at speed 10/100):
  1. read the current pose from /mycobot/angles_real
  2. J6 (tool roll) +25 deg, then back to the starting pose
  3. J1 (base yaw)  +20 deg, then back to the starting pose

The arm is left exactly in the pose it started in.

SAFETY (read docs/safety.md first):
  - Base clamped, arm near its upright pose, 350 mm exclusion zone clear.
  - Keep a hand near the power switch.
  - Ctrl+C stops this script but does NOT release servo torque: the arm will
    hold its last commanded position.
  - Requires the topic bridge already running in another terminal:
      ros2 launch mycobot_communication communication_jsnn.launch.py
    (the shipped launch targets the JN serial port itself; check accepted
    arguments with --show-args if needed)

Usage (on the Jetson, after sourcing ROS2 and the workspace):
    python3 initial_tests/ros2_smoke_move.py
"""

import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from mycobot_interfaces.msg import MycobotAngles, MycobotSetAngles
except ImportError as exc:
    print("ERROR: cannot import ROS2 packages (%s)." % exc)
    print("Source the environment first:")
    print("  source /opt/ros/foxy/setup.bash")
    print("  source ~/colcon_ws/install/setup.bash")
    sys.exit(1)

REAL_TOPIC = "/mycobot/angles_real"
GOAL_TOPIC = "/mycobot/angles_goal"

SPEED = 10          # 1..100, deliberately slow for a first ROS2 motion test
TOOL_DELTA = 25.0   # deg, joint 6 (tool roll, no arm displacement)
BASE_DELTA = 20.0   # deg, joint 1 (base yaw)
TOL_DEG = 4.0       # considered "arrived" within this error
MOVE_TIMEOUT = 15.0 # seconds per move
START_TIMEOUT = 10.0  # seconds to wait for the first telemetry message

# Joint soft limits (deg): J1-J5 +-170, J6 +-165 (see docs/safety.md).
J_LIMITS = [(-170.0, 170.0)] * 5 + [(-165.0, 165.0)]


def fmt(angles):
    return "[" + ", ".join("%6.1f" % a for a in angles) + "]"


def clamp(value, low, high):
    return max(low, min(high, value))


class SmokeMove(Node):
    def __init__(self):
        super().__init__("mycobot_smoke_move")
        self.latest = None
        self.create_subscription(MycobotAngles, REAL_TOPIC, self._on_angles, 10)
        self.pub = self.create_publisher(MycobotSetAngles, GOAL_TOPIC, 10)

    def _on_angles(self, msg):
        self.latest = [msg.joint_1, msg.joint_2, msg.joint_3,
                       msg.joint_4, msg.joint_5, msg.joint_6]

    def spin_for(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

    def wait_for_pose(self, timeout):
        """Wait until at least one /mycobot/angles_real message arrived."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin_for(0.1)
            if self.latest is not None:
                return list(self.latest)
        return None

    def send(self, angles, speed=SPEED):
        msg = MycobotSetAngles()
        for i, value in enumerate(angles):
            setattr(msg, "joint_%d" % (i + 1), float(value))
        msg.speed = int(speed)
        self.pub.publish(msg)
        print("  sent: %s  (speed %d)" % (fmt(angles), speed))

    def wait_until_at(self, target, timeout=MOVE_TIMEOUT):
        """Poll telemetry until the arm is within TOL_DEG of target."""
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            self.spin_for(0.2)
            if self.latest is None:
                continue
            error = max(abs(a - b) for a, b in zip(self.latest, target))
            if error <= TOL_DEG:
                return True
        return False


def move_and_wait(node, target, label):
    """Send target and wait until reached. Returns True on success."""
    print("%s -> %s" % (label, fmt(target)))
    node.send(target)
    if node.wait_until_at(target):
        print("  reached: %s" % fmt(node.latest))
        return True
    print("  !! did not reach target within %.0f s (last: %s)"
          % (MOVE_TIMEOUT, fmt(node.latest) if node.latest else "no telemetry"))
    return False


def main():
    rclpy.init()
    node = SmokeMove()

    try:
        print("Waiting for telemetry on %s ..." % REAL_TOPIC)
        start = node.wait_for_pose(START_TIMEOUT)
        if start is None:
            print("ERROR: no telemetry within %.0f s." % START_TIMEOUT)
            print("Is the bridge running? Try:")
            print("  ros2 launch mycobot_communication communication_jsnn.launch.py")
            print("Then check:  ros2 topic list   /   ros2 topic echo %s --once" % REAL_TOPIC)
            return 1

        print("Current pose: %s" % fmt(start))
        print()
        print("Planned, at speed %d:" % SPEED)
        print("  1) J6 +%.0f deg, back to start" % TOOL_DELTA)
        print("  2) J1 +%.0f deg, back to start" % BASE_DELTA)
        print("Base clamped, arm upright, 350 mm zone clear, hand near the switch.")
        answer = input("Type MOVE to continue (anything else aborts): ").strip()
        if answer != "MOVE":
            print("Aborted - nothing was sent.")
            return 1

        moves = []
        for index, delta, label in ((5, TOOL_DELTA, "J6 tool roll"),
                                    (0, BASE_DELTA, "J1 base yaw")):
            target = list(start)
            low, high = J_LIMITS[index]
            candidate = clamp(start[index] + delta, low, high)
            if abs(candidate - start[index]) < TOL_DEG:
                # Already at the soft limit: nudge the other way instead.
                candidate = clamp(start[index] - delta, low, high)
            target[index] = candidate
            moves.append((index, target, label))

        for index, target, label in moves:
            print()
            if not move_and_wait(node, target, label):
                print("Returning to the starting pose before aborting...")
                move_and_wait(node, start, "recovery")
                return 1
            if not move_and_wait(node, start, "back to start"):
                return 1

        # Settle, then report.
        node.spin_for(0.5)
        print()
        print("Final pose: %s" % fmt(node.latest))
        print("Smoke test PASSED - arm left in the pose it started from.")
        print("Note: exiting this script does NOT release servo torque "
              "(see docs/safety.md 5.2).")
        return 0

    except KeyboardInterrupt:
        print("\nInterrupted. The arm keeps holding its last commanded position "
              "(Ctrl+C does not release torque).")
        return 130
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
