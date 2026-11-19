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
Run the fixed goal set and report the pose error of every goal (issue #140).

  ros2 run braco_mission run_goal_set.py
  ros2 run braco_mission run_goal_set.py --only line_y,pose_turn --out bags/real_1

Reads a goal set (default: config/goal_set.yaml of braco_mission). For each goal it first
sends the arm to the set's start pose (/move_to_pose), then runs the goal: a straight line
(/move_cartesian, the tcp read from TF every 20 ms as in draw_line.py) or a full pose
(/move_to_pose). The returns to the start are goals too and are reported as `start`.

Prints one line per goal and writes <out>/summary.csv (one row per goal: position and
orientation error, and for lines the planned and executed deviation from the straight line)
and <out>/<name>.csv/.png for every line. The default <out> is bags/goal_set: ros2_ws/bags is
not tracked by git. It stops at the first goal that fails to execute (a refused goal, a
partial plan or an execution error); a goal outside the tolerance is reported and it goes on.

The arm moves for every goal: on the real arm follow docs/guides/safety.md section 9, with a
hand near the power. It asks before the first motion; --yes skips the question.
"""

import argparse
import csv
import importlib.util
import math
import os
import sys

SAMPLE_PERIOD_S = 0.02
QUAT_NORM_TOLERANCE = 1e-3   # as the node (pose_math.hpp): far from unit length is a typo
SUMMARY_FIELDS = ['name', 'kind', 'success', 'position_error_mm', 'orientation_error_deg',
                  'fraction', 'planned_deviation_mm', 'executed_deviation_max_mm',
                  'executed_deviation_mean_mm', 'executed_end_error_mm', 'message']


def _load_draw_line():
    """Return draw_line.py as a module: the path geometry and plot are shared with it."""
    path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'draw_line.py')
    spec = importlib.util.spec_from_file_location('draw_line', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


draw_line = _load_draw_line()


# ---- the goal set (no ROS: test/test_run_goal_set.py) ------------------------------------

def _point(value, what):
    if (not isinstance(value, (list, tuple)) or len(value) != 3 or
            not all(isinstance(v, (int, float)) and math.isfinite(v) for v in value)):
        raise ValueError('%s: expected [x, y, z] in metres, got %r' % (what, value))
    return tuple(float(v) for v in value)


def _pose(value, what):
    if not isinstance(value, dict) or set(value) != {'position', 'orientation'}:
        raise ValueError('%s: expected {position: [x, y, z], orientation: [x, y, z, w]}' % what)
    q = value['orientation']
    if (not isinstance(q, (list, tuple)) or len(q) != 4 or
            not all(isinstance(v, (int, float)) and math.isfinite(v) for v in q)):
        raise ValueError('%s: orientation must be [x, y, z, w], got %r' % (what, q))
    if abs(math.sqrt(sum(v * v for v in q)) - 1.0) > QUAT_NORM_TOLERANCE:
        raise ValueError('%s: orientation %r is not a unit quaternion' % (what, q))
    return {'position': _point(value['position'], what + ' position'),
            'orientation': tuple(float(v) for v in q)}


def parse_goal_set(data):
    """
    Return the goal set checked: {'frame', 'start', 'goals'}.

    Each goal is {'name', 'kind': 'line' | 'pose', 'line': (x, y, z)} or {..., 'pose': pose}.
    Raises ValueError naming the first problem.
    """
    if not isinstance(data, dict):
        raise ValueError('the goal set must be a mapping with start and goals')
    frame = data.get('frame', 'world')
    start = _pose(data.get('start'), 'start')
    goals = []
    for i, goal in enumerate(data.get('goals') or []):
        if not isinstance(goal, dict) or not isinstance(goal.get('name'), str):
            raise ValueError('goal %d: needs a name' % i)
        name = goal['name']
        if name == 'start' or any(g['name'] == name for g in goals):
            raise ValueError('goal %s: the name is used twice (or is "start")' % name)
        kinds = [k for k in ('line', 'pose') if k in goal]
        if len(kinds) != 1 or set(goal) != {'name', kinds[0]}:
            raise ValueError('goal %s: needs exactly one of line or pose' % name)
        if kinds[0] == 'line':
            goals.append({'name': name, 'kind': 'line', 'line': _point(goal['line'], name)})
        else:
            goals.append({'name': name, 'kind': 'pose', 'pose': _pose(goal['pose'], name)})
    if not goals:
        raise ValueError('the goal set has no goals')
    return {'frame': frame, 'start': start, 'goals': goals}


def select(goals, only):
    """Return the goals named in `only` (comma-separated, set order), all when it is empty."""
    if not only:
        return goals
    names = [n.strip() for n in only.split(',') if n.strip()]
    unknown = [n for n in names if not any(g['name'] == n for g in goals)]
    if unknown:
        raise ValueError('no goal named %s' % ', '.join(unknown))
    return [g for g in goals if g['name'] in names]


def summary_row(name, kind, result, executed=None):
    """Return a summary.csv row: errors in mm and degrees, empty where they do not apply."""
    def mm(m):
        return '' if m is None or math.isnan(m) else '%.3f' % (m * 1000.0)

    row = dict.fromkeys(SUMMARY_FIELDS, '')
    row.update(name=name, kind=kind, success=bool(result.success),
               position_error_mm=mm(result.position_error_m),
               orientation_error_deg=('' if math.isnan(result.orientation_error_rad)
                                      else '%.3f' % math.degrees(result.orientation_error_rad)),
               message=result.message)
    if kind == 'line':
        row.update(fraction='%.3f' % result.fraction,
                   planned_deviation_mm=mm(result.max_deviation_m))
        if executed:
            row.update(executed_deviation_max_mm=mm(executed['max_deviation_m']),
                       executed_deviation_mean_mm=mm(executed['mean_deviation_m']),
                       executed_end_error_mm=mm(executed['end_error_m']))
    return row


def format_row(row):
    """Return one line for the terminal."""
    text = '%-12s %-5s %-5s pos %8s mm  orient %7s deg' % (
        row['name'], row['kind'], 'ok' if row['success'] else 'FAIL',
        row['position_error_mm'] or '-', row['orientation_error_deg'] or '-')
    if row['kind'] == 'line':
        text += '  fraction %s  planned dev %s mm  executed dev max %s mm' % (
            row['fraction'], row['planned_deviation_mm'] or '-',
            row['executed_deviation_max_mm'] or '-')
    return text


def write_summary(path, rows):
    with open(path, 'w', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=SUMMARY_FIELDS)
        writer.writeheader()
        writer.writerows(rows)


def moved(result):
    """Return whether the goal ran to the end: reached, or only outside the tolerance."""
    return result.success or result.message.startswith('outside the tolerance')


# ---- ROS ---------------------------------------------------------------------------------

class Runner:
    """Sends the goals of the set and samples the tcp from TF while a line runs."""

    def __init__(self, frame):
        import rclpy
        from braco_interfaces.action import MoveCartesian, MoveToPose
        from rclpy.action import ActionClient
        from tf2_ros import Buffer, TransformListener

        self.rclpy = rclpy
        self.frame = frame
        self.node = rclpy.create_node('run_goal_set')
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self.node)
        self.MoveCartesian = MoveCartesian
        self.MoveToPose = MoveToPose
        self.pose_client = ActionClient(self.node, MoveToPose, 'move_to_pose')
        self.line_client = ActionClient(self.node, MoveCartesian, 'move_cartesian')
        self.samples = []
        self.t0 = None

    def wait(self):
        """Return None when both actions and the tcp transform are there, else why not."""
        from rclpy.duration import Duration
        from rclpy.time import Time

        for client, name in ((self.pose_client, '/move_to_pose'),
                             (self.line_client, '/move_cartesian')):
            if not client.wait_for_server(timeout_sec=10.0):
                return 'no %s: is braco_mission move_to_pose.launch.py running?' % name
        deadline = self.node.get_clock().now() + Duration(seconds=3.0)
        while not self.tf_buffer.can_transform(self.frame, draw_line.TCP, Time()):
            if self.node.get_clock().now() > deadline:
                return 'no %s -> %s transform from TF' % (self.frame, draw_line.TCP)
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
        return None

    def _sample(self):
        from rclpy.time import Time
        from tf2_ros import TransformException

        try:
            t = self.tf_buffer.lookup_transform(self.frame, draw_line.TCP, Time())
        except TransformException:
            return
        p = t.transform.translation
        dt = (self.node.get_clock().now() - self.t0).nanoseconds * 1e-9
        self.samples.append((dt, p.x, p.y, p.z))

    def _send(self, client, goal):
        sent = client.send_goal_async(goal)
        self.rclpy.spin_until_future_complete(self.node, sent)
        handle = sent.result()
        if not handle.accepted:
            return None
        done = handle.get_result_async()
        self.rclpy.spin_until_future_complete(self.node, done)
        return done.result().result

    def pose(self, pose):
        """Run a /move_to_pose goal; return its result (None: rejected, another goal runs)."""
        goal = self.MoveToPose.Goal()
        goal.target.header.frame_id = self.frame
        p, q = goal.target.pose.position, goal.target.pose.orientation
        p.x, p.y, p.z = pose['position']
        q.x, q.y, q.z, q.w = pose['orientation']
        return self._send(self.pose_client, goal)

    def line(self, end):
        """Run a /move_cartesian line to `end`; return its result and the tcp samples."""
        from geometry_msgs.msg import Pose

        goal = self.MoveCartesian.Goal()
        goal.header.frame_id = self.frame
        waypoint = Pose()
        waypoint.position.x, waypoint.position.y, waypoint.position.z = end
        waypoint.orientation.w = 0.0   # all 0: keep the tcp orientation at the start
        goal.waypoints = [waypoint]
        self.samples = []
        self.t0 = self.node.get_clock().now()
        self._sample()
        timer = self.node.create_timer(SAMPLE_PERIOD_S, self._sample)
        try:
            result = self._send(self.line_client, goal)
        finally:
            timer.cancel()
            self.node.destroy_timer(timer)
        self._sample()
        return result, self.samples

    def close(self):
        self.node.destroy_node()


def run_line(runner, goal, out):
    """Run one line goal; write its CSV and plot. Return (summary row, it ran to the end)."""
    result, samples = runner.line(goal['line'])
    if result is None:
        return None, False
    executed = None
    if moved(result) and samples:
        a, b = samples[0][1:], goal['line']
        rows = draw_line.analyse(samples, a, b)
        executed = draw_line.summary(rows, a, b)
        path = os.path.join(out, goal['name'])
        draw_line.write_csv(path + '.csv', rows)
        draw_line.plot(path + '.png', rows, a, b,
                       '%s: %.0f mm, fraction %.2f, max deviation %.2f mm (executed, from TF)' % (
                           goal['name'], executed['length_m'] * 1000, result.fraction,
                           executed['max_deviation_m'] * 1000))
    return summary_row(goal['name'], 'line', result, executed), moved(result)


def run(args, goal_set):
    import rclpy

    goals = select(goal_set['goals'], args.only)
    print('%d goals from %s, each from the start pose:' % (len(goals), args.set))
    for g in goals:
        print('  %-12s %s' % (g['name'], g['kind']))
    if not args.yes:
        if input('The arm moves now. Continue? [y/N] ').strip().lower() != 'y':
            print('nothing sent')
            return 1

    os.makedirs(args.out, exist_ok=True)
    rows = []
    rclpy.init()
    runner = Runner(goal_set['frame'])
    try:
        why = runner.wait()
        if why:
            print(why, file=sys.stderr)
            return 1
        for goal in goals:
            result = runner.pose(goal_set['start'])
            if result is None:
                print('goal rejected: another goal is running', file=sys.stderr)
                break
            rows.append(summary_row('start', 'pose', result))
            print(format_row(rows[-1]))
            if not moved(result):
                break
            if goal['kind'] == 'line':
                row, ran = run_line(runner, goal, args.out)
            else:
                result = runner.pose(goal['pose'])
                row = None if result is None else summary_row(goal['name'], 'pose', result)
                ran = result is not None and moved(result)
            if row is None:
                print('goal rejected: another goal is running', file=sys.stderr)
                break
            rows.append(row)
            print(format_row(row))
            if not ran:
                print('%s did not run to the end: %s' % (goal['name'], row['message']),
                      file=sys.stderr)
                break
    finally:
        runner.close()
        rclpy.try_shutdown()
        if rows:
            write_summary(os.path.join(args.out, 'summary.csv'), rows)
            print('wrote %s' % os.path.join(args.out, 'summary.csv'))

    done = [r for r in rows if r['name'] != 'start']
    ok = len(done) == len(goals) and all(r['success'] for r in rows)
    verdict = 'all within the tolerance' if ok else 'NOT all ok'
    print('%d of %d goals sent, %s' % (len(done), len(goals), verdict))
    return 0 if ok else 1


def default_set():
    from ament_index_python.packages import get_package_share_directory
    return os.path.join(get_package_share_directory('braco_mission'), 'config', 'goal_set.yaml')


def main():
    import yaml
    from rclpy.utilities import remove_ros_args

    parser = argparse.ArgumentParser(prog='run_goal_set', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--set', help='goal set YAML (default: config/goal_set.yaml)')
    parser.add_argument('--only', default='',
                        help='comma-separated goal names to run (default: all)')
    parser.add_argument('--out', default=os.path.join('bags', 'goal_set'),
                        help='output folder (default bags/goal_set)')
    parser.add_argument('--yes', action='store_true', help='do not ask before moving the arm')
    args = parser.parse_args(remove_ros_args(sys.argv)[1:])
    args.set = args.set or default_set()
    try:
        with open(args.set) as f:
            goal_set = parse_goal_set(yaml.safe_load(f))
        select(goal_set['goals'], args.only)
    except (OSError, ValueError, yaml.YAMLError) as e:
        parser.error(str(e))
    return run(args, goal_set)


if __name__ == '__main__':
    sys.exit(main())
