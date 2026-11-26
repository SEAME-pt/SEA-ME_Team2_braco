#!/usr/bin/env python3
"""Clients, faults and a stop-time meter for the fault rehearsal (issue #142, make fault).

scripts/fault.sh runs it inside the system container (the repository is mounted on /opt/braco),
except `analyse`, which needs no ROS and runs on the host.

    stream      a client: takes the control (--take) and resends one target at --rate, like the
                page. Killing it is a client that dies mid-move. --nan sends NaN angles
    trajectory  the mission path: takes the control as `mission` and sends one goal to
                arm_controller (joint --joint moves by --delta deg in --seconds). Killing it is
                move_group dying mid-trajectory
    watch       records /joint_states, one line per message: wall time and the six angles (rad).
                --action stop|deactivate --after S sends a STOP or deactivates arm_driver S s
                after the first message, and marks it in the record
    pose        prints the six angles now, in degrees
    analyse     reads a watch record: was the arm moving at the fault, how long it took to stop
                after it, and how far it went (from --since, else the record's action, else its
                start)

Times are wall clock (time.time()): the container and the host share the clock, so a fault
injected from the host (date +%s.%N) and the record line up.
"""

import argparse
import math
import sys
import time

STILL_DEG = 0.3  # closer than this to a pose is the same pose (encoder noise)
STILL_WINDOW_S = 0.3  # the pose holds this long: the arm has stopped. A window, not a step between
# two messages: at 20 Hz a slow joint (5 deg/s) moves less than STILL_DEG per message
MOVING_WINDOW_S = 1.0  # motion this long before the fault counts as "moving at the fault"


# ---- Analysis (no ROS) ------------------------------------------------------------------


def step_deg(a, b):
    """Largest joint change between two poses in radians, in degrees."""
    return max(abs(math.degrees(y - x)) for x, y in zip(a, b))


def load_record(lines):
    """Samples [(t, angles)] and marks {name: t} of a watch record."""
    samples, marks = [], {}
    for line in lines:
        parts = line.split()
        if not parts:
            continue
        if parts[0] == '#':
            if len(parts) >= 3:
                try:
                    marks.setdefault(parts[1], float(parts[2]))
                except ValueError:
                    pass
            continue
        try:
            values = [float(v) for v in parts]
        except ValueError:
            continue  # a ROS log line mixed into the output
        if len(values) == 7:
            samples.append((values[0], values[1:]))
    return samples, marks


def moving_at(samples, t0, still_deg=STILL_DEG, window=MOVING_WINDOW_S):
    """True when the pose changed by still_deg or more in [t0 - window, t0]."""
    poses = [a for t, a in samples if t0 - window <= t <= t0]
    return any(step_deg(poses[0], a) >= still_deg for a in poses[1:])


def stop_ms(samples, t0, still_deg=STILL_DEG, window=STILL_WINDOW_S):
    """Milliseconds from t0 until the arm stopped, or None if it did not within the record.

    The arm stopped at the first sample (from the last one before t0) whose pose then holds,
    within still_deg, for window seconds. An upper bound: it may have arrived up to one
    /joint_states period (~50-70 ms) earlier.
    """
    k0 = max([k for k, (t, _) in enumerate(samples) if t <= t0], default=0)
    for k in range(k0, len(samples)):
        tk, pose = samples[k]
        if samples[-1][0] < tk + window:
            return None  # the record ends before the window does
        later = (a for t, a in samples[k + 1 :] if t <= tk + window)
        if all(step_deg(pose, a) < still_deg for a in later):
            return max(0.0, (tk - t0) * 1000.0)
    return None


def travel_deg(samples, t0):
    """Farthest any joint got, in degrees, from its pose at t0 (the last sample before it)."""
    before = [a for t, a in samples if t <= t0]
    start = before[-1] if before else samples[0][1]
    return max((step_deg(start, a) for t, a in samples if t >= t0), default=0.0)


def analyse(lines, since=None):
    """One line for fault.sh: 'moving=yes|no stop_ms=N|none latched_ms=N|none travel_deg=X'."""
    samples, marks = load_record(lines)
    if not samples:
        return 'moving=no stop_ms=none latched_ms=none travel_deg=0.0'
    t0 = next(t for t in (since, marks.get('action'), samples[0][0]) if t is not None)
    stopped = stop_ms(samples, t0)
    latched = marks.get('stopped')
    return 'moving={} stop_ms={} latched_ms={} travel_deg={:.1f}'.format(
        'yes' if moving_at(samples, t0) else 'no',
        'none' if stopped is None else f'{stopped:.0f}',
        'none' if latched is None or latched < t0 else f'{(latched - t0) * 1000.0:.0f}',
        travel_deg(samples, t0),
    )


# ---- ROS clients ----------------------------------------------------------------------------


def mark(name, t=None):
    print(f'# {name} {time.time() if t is None else t:.6f}', flush=True)


def take_control(node, source, force=True, release=False):
    import rclpy
    from braco_interfaces.srv import TakeControl

    cli = node.create_client(TakeControl, 'arm/take_control')
    if not cli.wait_for_service(timeout_sec=10.0):
        raise RuntimeError('arm/take_control not available: is arm_driver running?')
    req = TakeControl.Request(source=source, force=force, release=release)
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    res = fut.result()
    if not release and (res is None or not res.success):
        raise RuntimeError(f'take_control refused: {res.message if res else "no answer"}')


def current_pose(node, timeout=5.0):
    import rclpy
    from sensor_msgs.msg import JointState

    pose = []
    sub = node.create_subscription(
        JointState, 'joint_states', lambda m: pose or pose.extend(m.position[:6]), 10
    )
    end = time.monotonic() + timeout
    while not pose and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    if len(pose) != 6:
        raise RuntimeError('no /joint_states: is arm_driver running?')
    return pose


def wait_matched(node, pub, timeout=10.0):
    import rclpy

    end = time.monotonic() + timeout
    while pub.get_subscription_count() == 0 and time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.05)


def run_stream(node, args):
    import rclpy
    from braco_interfaces.msg import ArmCommand, JointTarget

    pub = node.create_publisher(ArmCommand, 'arm/command', 10)
    wait_matched(node, pub)
    if args.take:
        take_control(node, args.source)
    msg = ArmCommand(source=args.source)
    if args.all is not None:
        msg.type = ArmCommand.ALL_ANGLES
        msg.angles_deg = [math.nan if args.nan else a for a in args.all]
        msg.speed = args.speed
    else:
        msg.type = ArmCommand.JOINTS
        angle = math.nan if args.nan else args.angle
        msg.targets = [JointTarget(joint=args.joint, angle_deg=angle, speed=args.speed)]
    mark('streaming')
    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        pub.publish(msg)
        rclpy.spin_once(node, timeout_sec=1.0 / args.rate)


def run_trajectory(node, args):
    import rclpy
    from builtin_interfaces.msg import Duration
    from control_msgs.action import FollowJointTrajectory
    from rclpy.action import ActionClient
    from trajectory_msgs.msg import JointTrajectoryPoint

    start = current_pose(node)
    goal_pose = list(start)
    goal_pose[args.joint - 1] += math.radians(args.delta)
    client = ActionClient(node, FollowJointTrajectory, 'arm_controller/follow_joint_trajectory')
    if not client.wait_for_server(timeout_sec=10.0):
        raise RuntimeError('arm_controller is not running')
    take_control(node, 'mission')
    goal = FollowJointTrajectory.Goal()
    goal.trajectory.joint_names = [f'joint{i}' for i in range(1, 7)]
    whole, frac = divmod(args.seconds, 1.0)
    goal.trajectory.points = [
        JointTrajectoryPoint(
            positions=start, time_from_start=Duration(sec=0, nanosec=100_000_000)
        ),
        JointTrajectoryPoint(
            positions=goal_pose, time_from_start=Duration(sec=int(whole), nanosec=int(frac * 1e9))
        ),
    ]
    sent = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, sent, timeout_sec=10.0)
    handle = sent.result()
    if handle is None or not handle.accepted:
        raise RuntimeError('arm_controller refused the goal')
    mark('goal_accepted')
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result, timeout_sec=args.seconds + 10.0)
    status = result.result().status if result.done() else 'timeout'
    print(f'# result {status}', flush=True)
    take_control(node, 'mission', force=False, release=True)


def run_watch(node, args):
    import rclpy
    from braco_interfaces.msg import ArmCommand
    from lifecycle_msgs.msg import Transition
    from lifecycle_msgs.srv import ChangeState
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Bool

    state = {'first': None, 'acted': None}

    def on_joints(m):
        # arm_driver's stamp (wall clock): a probe that stalls must not shift the samples
        stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        t = stamp if stamp > 0 else time.time()
        if state['first'] is None:
            state['first'] = time.time()
            mark('ready', state['first'])
        print(f'{t:.6f} ' + ' '.join(f'{p:.6f}' for p in m.position[:6]), flush=True)

    def on_stopped(m):
        if m.data and state['acted'] is not None:
            mark('stopped')

    latched = QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )
    node.create_subscription(JointState, 'joint_states', on_joints, 10)
    node.create_subscription(Bool, 'arm/stopped', on_stopped, latched)
    pub = node.create_publisher(ArmCommand, 'arm/command', 10)
    change = node.create_client(ChangeState, 'arm_driver/change_state')
    if args.action == 'stop':
        wait_matched(node, pub)
    elif args.action == 'deactivate' and not change.wait_for_service(timeout_sec=10.0):
        raise RuntimeError('arm_driver/change_state not available')

    end = time.monotonic() + args.duration
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)
        due = state['first'] is not None and time.time() >= state['first'] + args.after
        if args.action and state['acted'] is None and due:
            state['acted'] = time.time()
            mark('action', state['acted'])
            if args.action == 'stop':
                pub.publish(ArmCommand(type=ArmCommand.STOP, source=args.source))
            else:
                req = ChangeState.Request()
                req.transition.id = Transition.TRANSITION_DEACTIVATE
                change.call_async(req)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest='cmd', required=True)

    s = sub.add_parser('stream')
    s.add_argument('--source', default='web')
    s.add_argument('--take', action='store_true', help='take the control (force) first')
    s.add_argument('--joint', type=int, default=1)
    s.add_argument('--angle', type=float, default=0.0)
    s.add_argument('--all', type=float, nargs=6, metavar='DEG', help='ALL_ANGLES instead')
    s.add_argument('--speed', type=int, default=10)
    s.add_argument('--nan', action='store_true', help='send NaN instead of the angles')
    s.add_argument('--rate', type=float, default=10.0)
    s.add_argument('--duration', type=float, default=120.0)

    t = sub.add_parser('trajectory')
    t.add_argument('--joint', type=int, default=1)
    t.add_argument('--delta', type=float, default=20.0, help='degrees')
    t.add_argument('--seconds', type=float, default=8.0)

    w = sub.add_parser('watch')
    w.add_argument('--duration', type=float, default=10.0)
    w.add_argument('--action', choices=['stop', 'deactivate'])
    w.add_argument('--after', type=float, default=2.0)
    w.add_argument('--source', default='fault')

    sub.add_parser('pose')

    a = sub.add_parser('analyse')
    a.add_argument('record')
    a.add_argument('--since', type=float, help='the fault (wall clock); default: the action mark')

    args = p.parse_args(argv)
    if args.cmd == 'analyse':
        with open(args.record) as f:
            print(analyse(f, args.since))
        return 0

    import rclpy

    rclpy.init()
    node = rclpy.create_node(f'fault_probe_{args.cmd}')
    try:
        if args.cmd == 'pose':
            print(' '.join(f'{math.degrees(a):.2f}' for a in current_pose(node)))
        else:
            {'stream': run_stream, 'trajectory': run_trajectory, 'watch': run_watch}[args.cmd](
                node, args
            )
    except KeyboardInterrupt:
        pass
    except RuntimeError as e:
        if not rclpy.ok():
            return 0  # SIGTERM: rclpy shut the context down under the loop (fault.sh's kill)
        print(f'fault_probe: {e}', file=sys.stderr)
        return 1
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
