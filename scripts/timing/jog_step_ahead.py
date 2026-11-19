#!/usr/bin/env python3
"""Joystick-style jog, reproducible: how smooth the arm moves (issue #93).

Does what joystick_teleop does with the stick held still (mycobot_cpp/includes/teleop_logic.hpp):
every --tick it sends JOINTS for one joint with target = latest measured angle + lead, at
--speed. It goes from the start angle to start + --range and back, --cycles times, then
reports how even the measured velocity was. The same motion for every arm_driver setting, so
runs can be compared (a person on the stick never repeats a move exactly).

  blue mode, full stick:    --speed 15 --lead 5      (SPEED_MODES[0])
  blue mode, ~25 % stick:   --speed 6  --lead 2

Metrics, on the cruise part of each segment (after 0.3 s, until the last lead of the way):
  v_mean   mean velocity (deg/s);  expected about speed * 1.52 deg/s
  cv_v     std / mean of the velocity between two /joint_states: 0 = perfectly even
  stall    share of /joint_states intervals slower than 20 % of v_mean
  cv_v_w   cv_v and stall over --window (default 0.2 s, #141), as cv_v_w and stall_pct_w: a slow
           joint moves only a few encoder steps between two /joint_states, so cv_v has a floor
  js_dt    /joint_states interval, median / p95 / max (ms); gaps > 250 ms counted as dropouts

The arm must be powered on, out of rest, in a pose where the joint can move --range degrees.
A STOP on /arm/command from any client ends the run at once.

    python3 jog_step_ahead.py --joint 1 --speed 15 --lead 5 --range 30 --label gap50 \\
        --out /ws/bags/timing/jog.jsonl
"""

import argparse
import json
import math
import statistics as st
import time

import rclpy
from braco_interfaces.msg import ArmCommand, JointTarget
from hold_load import SOURCE, publish_unless_stopped, release_control, take_control, watch_stop
from rclpy.node import Node
from sensor_msgs.msg import JointState
from stats import percentile, windowed_speeds

# JOINT_LIMITS of mycobot.hpp (degrees), minus a margin: the run never asks beyond them
LIMITS = [(-166, 166), (-138, 138), (-149, 149), (-149, 149), (-153, 158), (-178, 178)]
MARGIN_DEG = 5.0
HOLD_SPEED = 10  # teleop_logic.hpp: the speed that pins a joint at the end of a segment
# teleop_logic.hpp, advanceJogTarget (#93): --open-loop uses the same numbers
DEG_PER_S_PER_SPEED = 1.52
FEEDBACK_LAG_S = 0.2
BREAKAWAY_DEG = 3.0


def advance_jog_target(previous, measured, direction, speed, lead, remaining, dt, vel_scale=1.0):
    """Python copy of mycobot::teleop::advanceJogTarget: open-loop target, bounded."""
    velocity = speed * DEG_PER_S_PER_SPEED * vel_scale
    reach = max(0.0, min(lead + velocity * FEEDBACK_LAG_S, remaining))
    ahead = (
        BREAKAWAY_DEG if previous is None else (previous - measured) * direction + velocity * dt
    )
    return measured + direction * min(max(ahead, 0.0), reach)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--joint', type=int, default=1)
    ap.add_argument('--speed', type=int, default=15)
    ap.add_argument('--lead', type=float, default=5.0, help='degrees ahead of the measured angle')
    ap.add_argument('--range', type=float, default=30.0, help='degrees, signed: direction first')
    ap.add_argument('--cycles', type=int, default=2)
    ap.add_argument('--tick', type=float, default=0.02, help='s, joystick_teleop LOOP_PERIOD')
    ap.add_argument('--label', default='')
    ap.add_argument(
        '--vel-scale',
        type=float,
        default=1.0,
        help='--open-loop: target velocity = speed * 1.52 deg/s * this',
    )
    ap.add_argument(
        '--open-loop',
        action='store_true',
        help='the #93 teleop: target advances open loop (advanceJogTarget), not measured + lead',
    )
    ap.add_argument('--window', type=float, default=0.2, help='s, for cv_v_w and stall_pct_w')
    ap.add_argument('--out', help='JSON lines file to append the result to')
    ap.add_argument('--raw', help='JSON lines file to append the samples and segments to')
    args = ap.parse_args()
    j = args.joint - 1

    rclpy.init()
    node = Node('jog_step_ahead')
    pub = node.create_publisher(ArmCommand, 'arm/command', 10)
    stop = watch_stop(node)
    samples = []  # (header.stamp s, angle deg)
    node.create_subscription(
        JointState,
        'joint_states',
        lambda m: samples.append(
            (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9, math.degrees(m.position[j]))
        ),
        50,
    )
    end = time.monotonic() + 5.0
    while not samples and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not samples:
        raise RuntimeError('no /joint_states within 5 s')
    start = samples[-1][1]
    far = start + args.range
    lo, hi = LIMITS[j]
    if not lo + MARGIN_DEG <= far <= hi - MARGIN_DEG:
        raise RuntimeError(f'J{args.joint}: {start:.1f} + {args.range:g} leaves the limits')

    def send(angle, speed):
        cmd = ArmCommand(
            type=ArmCommand.JOINTS,
            source=SOURCE,
            targets=[JointTarget(joint=args.joint, angle_deg=round(angle, 1), speed=speed)],
        )
        return publish_unless_stopped(pub, cmd, stop)

    def spin_until(t):
        while not stop['stop'] and time.monotonic() < t:
            rclpy.spin_once(node, timeout_sec=max(0.0, t - time.monotonic()))

    segments = []  # (t_start, t_cruise_end) on the header.stamp clock
    # A segment slower than a third of the expected speed has stalled (a lead inside the servo's
    # dead zone holds the joint still): the run ends there instead of sending forever
    seg_timeout = 3 * abs(args.range) / (args.speed * 1.52) + 2.0
    stalled = None  # degrees left when a segment stalled
    take_control(node)
    try:
        for goal in [far, start] * args.cycles:
            direction = 1 if goal > samples[-1][1] else -1
            t_seg = time.time()
            t_cruise_end = None
            nxt = time.monotonic()
            target = None  # --open-loop: the jog target, None until the first tick
            t_prev = nxt
            deadline = nxt + seg_timeout
            while not stop['stop']:
                angle = samples[-1][1]
                remaining = (goal - angle) * direction
                if remaining <= 1.0:
                    break
                if time.monotonic() > deadline:
                    stalled = round(remaining, 1)
                    break
                if remaining <= args.lead and t_cruise_end is None:
                    t_cruise_end = time.time()
                if args.open_loop:
                    now = time.monotonic()
                    target = advance_jog_target(
                        target,
                        angle,
                        direction,
                        args.speed,
                        args.lead,
                        remaining,
                        now - t_prev,
                        args.vel_scale,
                    )
                    t_prev = now
                    send(target, args.speed)
                else:
                    send(angle + direction * min(args.lead, remaining), args.speed)
                nxt += args.tick
                spin_until(nxt)
            segments.append((t_seg + 0.3, t_cruise_end or time.time()))
            if stalled is not None:
                print(f'J{args.joint} stalled {stalled} deg short of {goal:.1f}: run ended')
                break
            # Pin the end, as the teleop does on release, and let the joint settle
            for _ in range(int(0.6 / args.tick)):
                if not send(goal, HOLD_SPEED):
                    break
                nxt += args.tick
                spin_until(nxt)
        if stop['stop']:
            print('STOP seen on /arm/command: run ended, nothing written')
            return
    finally:
        release_control(node)
        node.destroy_node()
        rclpy.shutdown()

    vel, dts, vmeans, vel_w = [], [], [], []
    for ta, tb in segments:
        pts = [p for p in samples if ta <= p[0] <= tb]
        if len(pts) < 4:
            continue
        v = [(b[1] - a[1]) / (b[0] - a[0]) for a, b in zip(pts, pts[1:]) if b[0] > a[0]]
        vm = (pts[-1][1] - pts[0][1]) / (pts[-1][0] - pts[0][0])
        vel += [abs(x) for x in v]
        vel_w += windowed_speeds(pts, args.window)
        vmeans.append(abs(vm))
        dts += [(b[0] - a[0]) * 1e3 for a, b in zip(pts, pts[1:])]
    if not vel and stalled is None:
        raise RuntimeError('no cruise samples: is the arm powered on and out of rest mode?')
    if not vel:  # stalled before any cruise: the joint never got going
        vel, vmeans, dts = (
            [0.0],
            [0.0],
            [(b[0] - a[0]) * 1e3 for a, b in zip(samples, samples[1:])],
        )
    v_mean = st.mean(vmeans)
    all_dts = [(b[0] - a[0]) * 1e3 for a, b in zip(samples, samples[1:])]
    result = {
        'label': args.label,
        'joint': args.joint,
        'speed': args.speed,
        'lead': args.lead,
        'open_loop': args.open_loop,
        'vel_scale': args.vel_scale,
        'expected_v': round(args.speed * 1.52, 1),
        'v_mean': round(v_mean, 1),
        'cv_v': round(st.pstdev(vel) / v_mean, 3) if v_mean else None,
        'stall_pct': round(100 * sum(1 for x in vel if x < 0.2 * v_mean) / len(vel), 1),
        'window_s': args.window,
        'cv_v_w': round(st.pstdev(vel_w) / v_mean, 3) if v_mean and vel_w else None,
        'stall_pct_w': (
            round(100 * sum(1 for x in vel_w if x < 0.2 * v_mean) / len(vel_w), 1)
            if vel_w
            else None
        ),
        'js_dt_p50': round(percentile(sorted(dts), 0.5), 1),
        'js_dt_p95': round(percentile(sorted(dts), 0.95), 1),
        'js_dt_max': round(max(dts), 1),
        'dropouts': sum(1 for d in all_dts if d > 250),
        'segments': len(vmeans),
        'stalled_deg_left': stalled,
    }
    print(json.dumps(result))
    if args.out:
        with open(args.out, 'a') as f:
            f.write(json.dumps(result) + '\n')
    if args.raw:
        with open(args.raw, 'a') as f:
            raw = {
                'label': args.label,
                'joint': args.joint,
                'speed': args.speed,
                'segments': segments,
                'samples': samples,
            }
            f.write(json.dumps(raw) + '\n')


if __name__ == '__main__':
    main()
