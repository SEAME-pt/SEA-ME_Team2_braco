#!/usr/bin/env python3
"""How long the arm takes to stand still after a move, and where it stops (issue #141).

For pick and place: the gripper must act only once the arm stands still, and a fixed sleep is
either too long (clean stop) or too short (a long move that swings). This measures both through
the same path MoveIt uses: FollowJointTrajectory on arm_controller -> BracoSystem -> arm_driver.

Moves, all joint-space and within +-30 deg of zero (the jog tests' range):
  size     zero -> short / medium / long target and back, --repeat times each
  free     medium target, reached alternately from zero and from the opposite side
  oneway   the same, but always through a point --bias deg past the target on the + side
           (backlash always taken up the same way)
  speed    zero -> long target and back at each of --speeds deg/s, twice: how even each joint
           moves during a multi-joint trajectory (cv of its velocity over a 200 ms window)

Per arrival: settle_s at 0.1 and 0.2 deg (from the trajectory's nominal end: what a pick and
place would wait after the move, see stats.settle_time), the rest angles and their error to the
target. Prints a summary; --out appends one JSON line per arrival.

The arm must be at zero, powered on, out of rest (make zero). A STOP from any client ends it.

    python3 settle_test.py --out /ws/bags/timing/settle.jsonl --label load
"""

import argparse
import json
import math
import statistics as st
import time

import rclpy
from braco_interfaces.srv import TakeControl
from control_msgs.action import FollowJointTrajectory
from hold_load import watch_stop
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from stats import settle_time, windowed_speeds
from trajectory_msgs.msg import JointTrajectoryPoint

JOINTS = [f'joint{i}' for i in range(1, 7)]
SOURCE = 'mission'  # BracoSystem only commands while the control owner is "mission"
ZERO = [0.0] * 6
TARGETS = {
    'short': [5, -8, -8, 5, 8, 0],
    'medium': [12, -18, -15, 10, 18, 0],
    'long': [20, -25, -25, 15, 30, 0],
}
OPPOSITE = [-12, 12, 12, -10, -18, 0]
DEG_PER_S = 20.0  # trajectory speed of the largest joint move
RECORD_S = 1.5  # samples kept after the nominal end of each move


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--repeat', type=int, default=4)
    ap.add_argument('--bias', type=float, default=3.0, help='deg past the target (oneway)')
    ap.add_argument('--speeds', default='8,20,40', help='deg/s of the largest joint (speed block)')
    ap.add_argument('--blocks', default='size,free,oneway,speed')
    ap.add_argument('--raw', help='JSON lines file for the samples of each speed-block move')
    ap.add_argument('--label', default='')
    ap.add_argument('--out', help='JSON lines file to append one line per arrival to')
    args = ap.parse_args()

    rclpy.init()
    node = Node('settle_test')
    stop = watch_stop(node)
    samples = []  # (header.stamp s, [deg] * 6)
    node.create_subscription(
        JointState,
        'joint_states',
        lambda m: samples.append(
            (
                m.header.stamp.sec + m.header.stamp.nanosec * 1e-9,
                [math.degrees(p) for p in m.position[:6]],
            )
        ),
        50,
    )
    client = ActionClient(node, FollowJointTrajectory, '/arm_controller/follow_joint_trajectory')
    control = node.create_client(TakeControl, 'arm/take_control')

    def wait(fut, timeout):
        rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
        return fut.done()

    def take(release=False):
        if not control.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('arm/take_control not available: is arm_driver running?')
        fut = control.call_async(TakeControl.Request(source=SOURCE, force=False, release=release))
        if not wait(fut, 5.0) and not release:
            raise RuntimeError('take_control: no answer')
        if not release and not fut.result().success:
            raise RuntimeError(f'take_control refused: {fut.result().message}')

    def spin_for(seconds):
        end = time.monotonic() + seconds
        while not stop['stop'] and time.monotonic() < end:
            rclpy.spin_once(node, timeout_sec=0.01)

    def move(target, seconds=None, deg_per_s=DEG_PER_S):
        """Send one trajectory; returns its nominal end (stamp clock) or None."""
        current = samples[-1][1]
        if seconds is None:
            seconds = max(1.0, max(abs(t - c) for t, c in zip(target, current)) / deg_per_s)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = [math.radians(d) for d in target]
        point.time_from_start.sec = int(seconds)
        point.time_from_start.nanosec = int((seconds % 1) * 1e9)
        goal.trajectory.points = [point]
        sent = time.time()
        handle = client.send_goal_async(goal)
        if not wait(handle, 5.0) or not handle.result().accepted:
            raise RuntimeError('trajectory goal refused: is arm_controller active?')
        result = handle.result().get_result_async()
        wait(result, seconds + 10.0)
        if stop['stop']:
            return None
        code = result.result().result.error_code if result.done() else 'timeout'
        if code != FollowJointTrajectory.Result.SUCCESSFUL:
            print(f'trajectory to {target} ended with {code}')
        return sent + seconds

    rows = []

    def arrive(target, block, name, seconds=None, deg_per_s=DEG_PER_S):
        start_pose = samples[-1][1]
        end = move(target, seconds, deg_per_s)
        if end is None:
            return False
        spin_for(max(0.0, end + RECORD_S - time.time()))
        window = [s for s in samples if s[0] >= end - 0.2]
        if len(window) < 5:
            print(f'{name}: too few /joint_states after the move')
            return True
        s01, s02 = settle_time(window, end, 0.1), settle_time(window, end, 0.2)
        tail = [a for t, a in window if t >= window[-1][0] - 0.3]
        rest = [st.mean(col) for col in zip(*tail)]
        row = {
            'label': args.label,
            'block': block,
            'move': name,
            'settle_s_01': None if s01 is None else round(s01, 3),
            'settle_s_02': None if s02 is None else round(s02, 3),
            'rest_deg': [round(r, 2) for r in rest],
            'err_deg': [round(r - t, 2) for r, t in zip(rest, target)],
        }
        if block == 'speed':
            duration = max(1.0, max(abs(t - c) for t, c in zip(target, start_pose)) / deg_per_s)
            t_a, t_b = end - duration + 0.3, end - 0.2  # cruise: skip the start and the end
            cvs = []
            for j in range(6):
                pts = [(t, a[j]) for t, a in samples if t_a <= t <= t_b]
                if abs(target[j] - start_pose[j]) < 5.0 or len(pts) < 5:
                    cvs.append(None)  # the joint barely moves: no velocity to judge
                    continue
                v = windowed_speeds(pts, 0.2)
                mean = abs(pts[-1][1] - pts[0][1]) / (pts[-1][0] - pts[0][0])
                cvs.append(round(st.pstdev(v) / mean, 3) if v and mean else None)
            row['deg_per_s'] = deg_per_s
            row['cv_w'] = cvs
            if args.raw:
                with open(args.raw, 'a') as f:
                    raw = [s for s in samples if end - duration - 0.2 <= s[0] <= end + RECORD_S]
                    f.write(json.dumps({**row, 'samples': raw}) + '\n')
        rows.append(row)
        if args.out:
            with open(args.out, 'a') as f:
                f.write(json.dumps(row) + '\n')
        return True

    end = time.monotonic() + 5.0
    while not samples and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not samples or max(abs(a) for a in samples[-1][1][:5]) > 2.0:
        raise RuntimeError('start at zero (make zero): no /joint_states or the arm is elsewhere')
    if not client.wait_for_server(timeout_sec=5.0):
        raise RuntimeError('/arm_controller/follow_joint_trajectory not available')

    biased = [t + (args.bias if i < 5 else 0.0) for i, t in enumerate(TARGETS['medium'])]
    take()
    try:
        blocks = args.blocks.split(',')
        plan = []
        for _ in range(args.repeat if 'size' in blocks else 0):
            for size in ('short', 'medium', 'long'):
                plan += [(TARGETS[size], 'size', size), (ZERO, 'size', f'{size}->zero')]
        for _ in range(args.repeat + 1 if 'free' in blocks else 0):
            plan += [
                (TARGETS['medium'], 'free', 'from_zero'),
                (OPPOSITE, 'free', 'to_opposite'),
                (TARGETS['medium'], 'free', 'from_opposite'),
                (ZERO, 'free', 'to_zero'),
            ]
        for _ in range(args.repeat + 1 if 'oneway' in blocks else 0):
            for origin, tag in ((ZERO, 'from_zero'), (OPPOSITE, 'from_opposite')):
                plan += [(origin, 'oneway', f'to_{tag[5:]}'), ('BIAS', 'oneway', 'bias')]
                plan += [(TARGETS['medium'], 'oneway', tag)]
        for speed in [float(x) for x in args.speeds.split(',')] if 'speed' in blocks else []:
            for _ in range(2):
                plan += [(TARGETS['long'], 'speed', f'out@{speed:g}', speed)]
                plan += [(ZERO, 'speed', f'back@{speed:g}', speed)]
        plan.append((ZERO, 'end', 'zero'))
        for step in plan:
            target, block, name = step[:3]
            if stop['stop']:
                break
            if target == 'BIAS':
                if move(biased) is None:
                    break
                continue
            if not arrive(target, block, name, deg_per_s=step[3] if len(step) > 3 else DEG_PER_S):
                break
    finally:
        take(release=True)
        node.destroy_node()
        rclpy.shutdown()

    if stop['stop']:
        print('STOP seen: run ended')

    def summary(sel, what):
        r = [x for x in rows if sel(x)]
        if not r:
            return
        s1 = [x['settle_s_01'] for x in r if x['settle_s_01'] is not None]
        s2 = [x['settle_s_02'] for x in r if x['settle_s_02'] is not None]
        never = sum(1 for x in r if x['settle_s_01'] is None)
        print(
            f'{what:<28} n={len(r):>2}  settle@0.1 mean {st.mean(s1) if s1 else float("nan"):.3f} '
            f'max {max(s1) if s1 else float("nan"):.3f} never {never}  '
            f'settle@0.2 max {max(s2) if s2 else float("nan"):.3f}'
        )

    for size in ('short', 'medium', 'long'):
        summary(lambda x, s=size: x['move'] == s, f'size {size}')
    for block in ('free', 'oneway'):
        r = [x for x in rows if x['block'] == block and x['move'].startswith('from_')]
        if r:
            spread = [max(x['rest_deg'][j] for x in r) - min(x['rest_deg'][j] for x in r)
                      for j in range(6)]
            print(f'{block:<8} rest spread per joint (deg): {[round(s, 2) for s in spread]}')
            summary(lambda x, b=block: x['block'] == b and x['move'].startswith('from_'), block)
    for x in rows:
        if x['block'] == 'speed':
            cv = ' '.join('  -  ' if c is None else f'{c:.3f}' for c in x['cv_w'])
            print(f"speed {x['move']:<10} cv@200ms J1..J6: {cv}   settle@0.1 {x['settle_s_01']}")


if __name__ == '__main__':
    main()
