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
Draw a straight line with the tcp and plot the path the arm really ran (issue #85).

  ros2 run braco_mission draw_line.py 0.2167 0.0654 0.1126
  ros2 run braco_mission draw_line.py 0.2167 0.0654 0.1126 --eef-step 0.002 --out bags/line

Sends one goal to /move_cartesian (the move_to_pose node of braco_mission): a straight line
from where the tcp is to X Y Z (world frame, metres), keeping the tcp orientation. While the
arm moves the tcp is read from TF (world -> tcp) every 20 ms. Then it prints the result of
the goal and the largest distance of the executed path from the ideal line, and writes
<out>.csv (t, x, y, z, along, deviation) and <out>.png (top and side views, deviation along
the line). The default <out> is bags/draw_line: ros2_ws/bags is not tracked by git.
"""

import argparse
import csv
import math
import os
import sys

FRAME = 'world'
TCP = 'tcp'
SAMPLE_PERIOD_S = 0.02
MIN_LENGTH_M = 0.001   # a shorter line is the tcp already at the end: nothing to draw


# ---- path geometry (no ROS: test/test_draw_line.py) -------------------------------------

def _sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def along(p, a, b):
    """Return how far the projection of p on the line a-b is from a (m), clamped to it."""
    ab = _sub(b, a)
    length = math.sqrt(_dot(ab, ab))
    if length == 0.0:
        return 0.0
    return min(max(_dot(_sub(p, a), ab) / length, 0.0), length)


def distance_to_segment(p, a, b):
    """Return the distance from p to the segment a-b (to a when a and b are the same point)."""
    ab = _sub(b, a)
    length2 = _dot(ab, ab)
    t = 0.0 if length2 == 0.0 else min(max(_dot(_sub(p, a), ab) / length2, 0.0), 1.0)
    closest = (a[0] + t * ab[0], a[1] + t * ab[1], a[2] + t * ab[2])
    return math.dist(p, closest)


def too_short(a, b):
    """Return why the line a-b cannot be drawn, or None when it can."""
    length = math.dist(a, b)
    if length < MIN_LENGTH_M:
        return 'the tcp is already %.1f mm from the end: move it first (move_to_pose)' % (
            length * 1000.0)
    return None


def analyse(samples, a, b):
    """
    Return the rows of the executed path against the ideal line a-b.

    samples: (t, x, y, z) tuples. Each row is (t, x, y, z, along_m, deviation_m).
    """
    rows = []
    for t, x, y, z in samples:
        p = (x, y, z)
        rows.append((t, x, y, z, along(p, a, b), distance_to_segment(p, a, b)))
    return rows


def summary(rows, a, b):
    """Return the line length, the largest and the mean deviation (m) and the end error."""
    if not rows:
        return None
    deviations = [r[5] for r in rows]
    last = rows[-1]
    return {
        'length_m': math.dist(a, b),
        'max_deviation_m': max(deviations),
        'mean_deviation_m': sum(deviations) / len(deviations),
        'end_error_m': math.dist((last[1], last[2], last[3]), b),
        'samples': len(rows),
    }


def write_csv(path, rows):
    with open(path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['t_s', 'x_m', 'y_m', 'z_m', 'along_m', 'deviation_m'])
        for row in rows:
            writer.writerow(['%.4f' % row[0]] + ['%.6f' % v for v in row[1:]])


def plot(path, rows, a, b, title):
    """Write a PNG: top view (x-y), side view (along-z) and the deviation along the line."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    xs = [r[1] for r in rows]
    ys = [r[2] for r in rows]
    zs = [r[3] for r in rows]
    s = [r[4] * 1000.0 for r in rows]
    dev = [r[5] * 1000.0 for r in rows]

    fig, (top, side, err) = plt.subplots(1, 3, figsize=(15, 4.5))
    fig.suptitle(title)
    top.plot([a[0] * 1000, b[0] * 1000], [a[1] * 1000, b[1] * 1000], 'k--', label='ideal line')
    top.plot([x * 1000 for x in xs], [y * 1000 for y in ys], '.-', ms=3, label='tcp (TF)')
    top.set(xlabel='x (mm)', ylabel='y (mm)', title='Top view (world)')
    top.axis('equal')
    top.legend()
    side.plot([0, math.dist(a, b) * 1000], [a[2] * 1000, b[2] * 1000], 'k--', label='ideal line')
    side.plot(s, [z * 1000 for z in zs], '.-', ms=3, label='tcp (TF)')
    side.set(xlabel='along the line (mm)', ylabel='z (mm)', title='Side view')
    side.legend()
    err.plot(s, dev, '.-', ms=3)
    err.set(xlabel='along the line (mm)', ylabel='distance from the line (mm)',
            title='Deviation of the executed path')
    err.grid(True)
    fig.tight_layout()
    fig.savefig(path, dpi=120)
    plt.close(fig)


# ---- ROS ---------------------------------------------------------------------------------

def run(args):
    import rclpy
    from braco_interfaces.action import MoveCartesian
    from geometry_msgs.msg import Pose
    from rclpy.action import ActionClient
    from rclpy.duration import Duration
    from rclpy.time import Time
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init()
    node = rclpy.create_node('draw_line')
    tf_buffer = Buffer()
    TransformListener(tf_buffer, node)
    client = ActionClient(node, MoveCartesian, 'move_cartesian')
    samples = []
    start = node.get_clock().now()

    def sample():
        try:
            t = tf_buffer.lookup_transform(FRAME, TCP, Time())
        except TransformException:
            return
        p = t.transform.translation
        samples.append(((node.get_clock().now() - start).nanoseconds * 1e-9, p.x, p.y, p.z))

    try:
        if not client.wait_for_server(timeout_sec=10.0):
            print('no /move_cartesian: is braco_mission move_to_pose.launch.py running?',
                  file=sys.stderr)
            return 1
        if not tf_buffer.can_transform(FRAME, TCP, Time(), timeout=Duration(seconds=0)):
            deadline = node.get_clock().now() + Duration(seconds=3.0)
            while node.get_clock().now() < deadline and not tf_buffer.can_transform(
                    FRAME, TCP, Time()):
                rclpy.spin_once(node, timeout_sec=0.1)

        goal = MoveCartesian.Goal()
        goal.header.frame_id = FRAME
        waypoint = Pose()
        waypoint.position.x, waypoint.position.y, waypoint.position.z = args.x, args.y, args.z
        waypoint.orientation.w = 0.0   # all 0: keep the tcp orientation at the start
        goal.waypoints = [waypoint]
        goal.eef_step_m = args.eef_step

        start = node.get_clock().now()
        sample()
        if not samples:
            print('no %s -> %s transform from TF' % (FRAME, TCP), file=sys.stderr)
            return 1
        why = too_short(samples[0][1:], (args.x, args.y, args.z))
        if why:
            print(why, file=sys.stderr)
            return 1
        timer = node.create_timer(SAMPLE_PERIOD_S, sample)
        sent = client.send_goal_async(
            goal, feedback_callback=lambda f: print('  stage:', f.feedback.stage))
        rclpy.spin_until_future_complete(node, sent)
        handle = sent.result()
        if not handle.accepted:
            print('goal rejected: another goal is running', file=sys.stderr)
            return 1
        done = handle.get_result_async()
        rclpy.spin_until_future_complete(node, done)
        timer.cancel()
        sample()
        result = done.result().result
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    print('result: success %s, fraction %.3f, planned deviation %.3f mm, %s' % (
        result.success, result.fraction, result.max_deviation_m * 1000.0, result.message))
    a = samples[0][1:]
    b = (args.x, args.y, args.z)
    rows = analyse(samples, a, b)
    s = summary(rows, a, b)
    print('executed: line %.1f mm, %d samples, deviation max %.3f mm, mean %.3f mm, '
          'end error %.3f mm' % (s['length_m'] * 1000, s['samples'],
                                 s['max_deviation_m'] * 1000, s['mean_deviation_m'] * 1000,
                                 s['end_error_m'] * 1000))
    out_dir = os.path.dirname(args.out)
    if out_dir:
        os.makedirs(out_dir, exist_ok=True)
    write_csv(args.out + '.csv', rows)
    plot(args.out + '.png', rows, a, b,
         'Straight line %.0f mm, fraction %.2f, max deviation %.2f mm (executed, from TF)' % (
             s['length_m'] * 1000, result.fraction, s['max_deviation_m'] * 1000))
    print('wrote %s.csv and %s.png' % (args.out, args.out))
    return 0 if result.success else 1


def main():
    from rclpy.utilities import remove_ros_args

    parser = argparse.ArgumentParser(prog='draw_line', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    for axis in 'xyz':
        parser.add_argument(axis, type=float, help='end of the line, world %s (m)' % axis)
    parser.add_argument('--eef-step', type=float, default=0.0,
                        help="metres between two tcp points of the path (default: the node's)")
    parser.add_argument('--out', default=os.path.join('bags', 'draw_line'),
                        help='output path without extension (default bags/draw_line)')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    if args.eef_step < 0.0:
        parser.error('--eef-step must not be negative')
    return run(args)


if __name__ == '__main__':
    sys.exit(main())
