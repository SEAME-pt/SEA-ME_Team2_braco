#!/usr/bin/env python3
"""Command -> motion latency: /arm/command to the first changed joint reading (issue #90, m. 3).

Moves joint 6 back and forth by --delta degrees, --n times. Each step resends the target at
--rate (like a real client: the watchdog stops the arm after 300 ms without a command).
Latency = header.stamp of the first /joint_states whose joint 6 moved more than --threshold
from where it was, minus the time the first command of the step was published.
Resolution: one /joint_states period.

The arm must be powered on in a safe pose (zero) first. Check the camera cable on J4-J6.
A STOP on /arm/command from any client ends the run; nothing is written to --out then.

    python3 latency_step.py --rate 20 --n 20 --out /ws/bags/timing/latency.md
"""

import argparse
import math
import os
import time

import rclpy
from braco_interfaces.msg import ArmCommand
from hold_load import (
    SOURCE,
    current_pose_deg,
    publish_unless_stopped,
    release_control,
    take_control,
    watch_stop,
)
from rclpy.node import Node
from sensor_msgs.msg import JointState
from stats import percentile

JOINT = 5  # joint 6, the wrist rotation: the lightest move


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--rate', type=float, default=20.0, help='command resend rate')
    ap.add_argument('--n', type=int, default=20)
    ap.add_argument('--delta', type=float, default=5.0, help='step size, degrees')
    ap.add_argument('--threshold', type=float, default=0.5, help='degrees that count as moved')
    ap.add_argument('--speed', type=int, default=30)
    ap.add_argument('--settle', type=float, default=1.5, help='seconds per step')
    ap.add_argument('--requested', type=float, default=float('nan'), help='state_rate_hz')
    ap.add_argument('--out', default='latency.md')
    args = ap.parse_args()

    rclpy.init()
    node = Node('latency_step')
    pub = node.create_publisher(ArmCommand, 'arm/command', 10)
    stop = watch_stop(node)
    take_control(node)
    try:  # from here on the control is ours: release it whatever happens
        base = current_pose_deg(node)

        latest = {}
        node.create_subscription(
            JointState,
            'joint_states',
            lambda m: latest.update(
                deg=math.degrees(m.position[JOINT]),
                stamp=m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
            ),
            10,
        )
        end = time.monotonic() + 5.0
        while 'deg' not in latest and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.1)
        if 'deg' not in latest:
            raise RuntimeError('no /joint_states within 5 s')
        clock = node.get_clock()
        lat_ms, missed = [], 0
        for i in range(args.n):
            if stop['stop']:
                break
            target = list(base)
            target[JOINT] += args.delta if i % 2 == 0 else 0.0
            cmd = ArmCommand(
                type=ArmCommand.ALL_ANGLES, angles_deg=target, speed=args.speed, source=SOURCE
            )
            rclpy.spin_once(node, timeout_sec=0.05)
            start_deg = latest['deg']
            t_send = clock.now().nanoseconds * 1e-9
            if not publish_unless_stopped(pub, cmd, stop):  # a STOP seen by the spin above
                break
            found = None
            nxt = time.monotonic() + 1.0 / args.rate
            end = time.monotonic() + args.settle
            while time.monotonic() < end and not stop['stop']:
                rclpy.spin_once(node, timeout_sec=0.005)
                if stop['stop']:
                    break
                if found is None and latest['stamp'] > t_send:
                    if abs(latest['deg'] - start_deg) > args.threshold:
                        found = (latest['stamp'] - t_send) * 1000.0
                if time.monotonic() >= nxt:
                    if not publish_unless_stopped(pub, cmd, stop):
                        break
                    nxt += 1.0 / args.rate
            if found is None:
                missed += 1
            else:
                lat_ms.append(found)
            print(
                f'step {i + 1}/{args.n}: '
                + (f'{found:.0f} ms' if found is not None else 'no motion seen')
            )
    finally:
        release_control(node)
        node.destroy_node()
        rclpy.shutdown()

    if stop['stop']:
        print('STOP seen on /arm/command: run ended, nothing written')
        return
    if not lat_ms:
        print('no step was seen: is the arm powered on and out of rest mode?')
        return
    s = sorted(lat_ms)
    header = (
        '| requested Hz | cmd Hz | steps | min ms | p50 ms | p99 ms | max ms | not seen |\n'
        '|---|---|---|---|---|---|---|---|\n'
    )
    row = (
        f'| {args.requested:g} | {args.rate:g} | {len(s)} | {s[0]:.0f} '
        f'| {percentile(s, 0.5):.0f} | {percentile(s, 0.99):.0f} | {s[-1]:.0f} | {missed} |\n'
    )
    new = not os.path.exists(args.out)
    with open(args.out, 'a') as f:
        f.write(header if new else '')
        f.write(row)
    print(header + row, end='')


if __name__ == '__main__':
    main()
