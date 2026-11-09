#!/usr/bin/env python3
"""Achieved /joint_states rate and jitter, plus arm_driver CPU (issue #90, measurement 2).

Intervals come from header.stamp (set by arm_driver right after the serial read), so the
subscriber's own delays do not count. CPU is read from /proc, so run it in the same
container as arm_driver.

    python3 js_rate.py --requested 50 --label idle --duration 60 --out /ws/bags/timing/rate.md

Appends one Markdown row to --out (writes the header first if the file is new).
"""

import argparse
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from stats import interval_stats

HEADER = (
    '| requested Hz | load | msgs | achieved Hz | mean ms | p50 ms | p99 ms | max ms '
    '| late > 1.5 T | arm_driver CPU % |\n'
    '|---|---|---|---|---|---|---|---|---|---|\n'
)


def driver_pid():
    for pid in filter(str.isdigit, os.listdir('/proc')):
        try:
            with open(f'/proc/{pid}/cmdline', 'rb') as f:
                if b'braco_driver/arm_driver' in f.read():
                    return int(pid)
        except OSError:
            continue
    return None


def cpu_seconds(pid):
    """utime + stime of the process in seconds, or None when it is gone."""
    try:
        with open(f'/proc/{pid}/stat') as f:
            fields = f.read().rsplit(')', 1)[1].split()
    except OSError:
        return None  # arm_driver exited or restarted during the run
    return (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--requested', type=float, required=True, help='state_rate_hz of arm_driver')
    ap.add_argument('--label', default='idle', help='e.g. idle, load')
    ap.add_argument('--duration', type=float, default=60.0)
    ap.add_argument('--out', default='rate.md')
    args = ap.parse_args()

    rclpy.init()
    node = Node('js_rate')
    stamps = []
    node.create_subscription(
        JointState,
        'joint_states',
        lambda m: stamps.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9),
        100,
    )

    pid = driver_pid()
    cpu0, t0 = (cpu_seconds(pid) if pid else None), time.monotonic()
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    cpu1 = cpu_seconds(pid) if pid else None
    cpu = (
        'n/a'
        if cpu0 is None or cpu1 is None
        else f'{100 * (cpu1 - cpu0) / (time.monotonic() - t0):.0f}'
    )
    node.destroy_node()
    rclpy.shutdown()

    if len(stamps) < 2:
        print('fewer than 2 messages on /joint_states: is arm_driver running?')
        return
    s = interval_stats(stamps, args.requested)
    row = (
        f'| {args.requested:g} | {args.label} | {len(stamps)} '
        f'| {s["achieved_hz"]:.1f} | {s["mean_ms"]:.1f} '
        f'| {s["p50_ms"]:.1f} | {s["p99_ms"]:.1f} | {s["max_ms"]:.1f} '
        f'| {s["late_pct"]:.1f} % | {cpu} |\n'
    )
    new = not os.path.exists(args.out)
    with open(args.out, 'a') as f:
        f.write(HEADER if new else '')
        f.write(row)
    print(HEADER + row, end='')


if __name__ == '__main__':
    main()
