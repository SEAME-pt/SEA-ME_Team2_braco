#!/usr/bin/env python3
"""The command load the #87 plugin will put on arm_driver (issue #90, measurement 2 "with #87").

ADR-0004: write() publishes ALL_ANGLES on /arm/command at state_rate_hz even when the target
does not change. This sends the CURRENT pose at --rate, so the arm holds still, but every
command still costs arm_driver a serial write and COMMAND_GAP.

The arm must be powered on in a safe pose (zero) first: holding a pose locks the joints.
A STOP on /arm/command from any client ends the run, as for the #87 plugin (ADR-0004).

    python3 hold_load.py --rate 50 --duration 70
"""

import argparse
import math
import time

import rclpy
from braco_interfaces.msg import ArmCommand
from braco_interfaces.srv import TakeControl
from rclpy.node import Node
from sensor_msgs.msg import JointState

SOURCE = 'script'


def take_control(node, release=False):
    cli = node.create_client(TakeControl, 'arm/take_control')
    if not cli.wait_for_service(timeout_sec=5.0):
        raise RuntimeError('arm/take_control not available: is arm_driver running?')
    fut = cli.call_async(TakeControl.Request(source=SOURCE, force=False, release=release))
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    res = fut.result()
    if release:
        return  # best effort: the control may already be lost (forced away, idle timeout)
    if res is None or not res.success:
        raise RuntimeError(f'take_control refused: {res.message if res else "no answer"}')


def watch_stop(node):
    """{'stop': True} once a STOP is seen on /arm/command, from any client: the run must end.

    arm_driver stops the arm on a STOP but keeps the control with its owner, so a client
    that resends its target would start the arm again on the next command.
    """
    seen = {'stop': False}

    def on_command(msg):
        if msg.type == ArmCommand.STOP:
            seen['stop'] = True

    node.create_subscription(ArmCommand, 'arm/command', on_command, 10)
    return seen


def current_pose_deg(node):
    pose = []
    sub = node.create_subscription(
        JointState, 'joint_states', lambda m: pose.extend(m.position[:6]) if not pose else None, 10
    )
    end = time.monotonic() + 5.0
    while not pose and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if len(pose) != 6:
        raise RuntimeError('no /joint_states within 5 s')
    return [math.degrees(p) for p in pose]


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--rate', type=float, required=True, help='commands per second')
    ap.add_argument('--duration', type=float, default=70.0)
    ap.add_argument('--speed', type=int, default=10)
    args = ap.parse_args()

    rclpy.init()
    node = Node('hold_load')
    pub = node.create_publisher(ArmCommand, 'arm/command', 10)
    stop = watch_stop(node)
    take_control(node)
    try:  # from here on the control is ours: release it whatever happens
        pose = current_pose_deg(node)
        print('holding', ' '.join(f'{a:.1f}' for a in pose), f'at {args.rate:g} Hz')
        cmd = ArmCommand(
            type=ArmCommand.ALL_ANGLES, angles_deg=pose, speed=args.speed, source=SOURCE
        )
        period = 1.0 / args.rate
        nxt = time.monotonic()
        end = nxt + args.duration
        while time.monotonic() < end and not stop['stop']:
            pub.publish(cmd)
            nxt += period
            while not stop['stop'] and time.monotonic() < nxt:
                rclpy.spin_once(node, timeout_sec=max(0.0, nxt - time.monotonic()))
        if stop['stop']:
            print('STOP seen on /arm/command: stopped sending')
    finally:
        take_control(node, release=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
