"""RosArm and the ROS panels without ROS: the bridge, messages and services are faked.

What matters most: a move is a stream of bounded steps resent faster than
arm_driver's 300 ms watchdog, STOP cancels it, and park / wake never power or rest the arm by themselves.
"""

import math
import threading
import time

import pytest

from web.server import arm, ros, ros_graph, ros_health


class FakeMsg:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class FakeArmCommand(FakeMsg):
    STOP = 0
    JOINTS = 1
    ALL_ANGLES = 2


class FakeSrv:
    class Request(FakeMsg):
        pass


class FakePublisher:
    def __init__(self):
        self.sent = []
        self.times = []

    def publish(self, msg):
        self.sent.append(msg)
        self.times.append(time.monotonic())


class FakeBridge:
    """Records service calls; every call succeeds unless `fail` names it."""

    def __init__(self):
        self.node = None
        self.available = False
        self.error = 'fake bridge'
        self.calls = []
        self.fail = set()

    def start(self):
        pass

    def when_ready(self, fn):
        pass

    def call(self, client, request, timeout=5.0):
        self.calls.append((client, getattr(request, 'data', None)))
        ok = client not in self.fail
        return FakeMsg(success=ok, message='' if ok else client + ' refused')


class Feeder:
    """Publishes /joint_states for `angles` every 20 ms, like arm_driver does."""

    def __init__(self, source, angles):
        self.source = source
        self.angles = list(angles)
        self._stop = threading.Event()
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def _run(self):
        while not self._stop.is_set():
            self.source._on_joint_state(joint_state(self.angles))
            time.sleep(0.02)

    def close(self):
        self._stop.set()
        self._thread.join()


def joint_state(degrees, names=None):
    return FakeMsg(
        name=names or ['joint{}'.format(i) for i in range(1, 7)],
        position=[math.radians(d) for d in degrees],
    )


@pytest.fixture
def ros_arm():
    bridge = FakeBridge()
    source = arm.RosArm(bridge=bridge)
    source._types = {
        'ArmCommand': FakeArmCommand,
        'ColorRGBA': FakeMsg,
        'SetBool': FakeSrv,
        'Trigger': FakeSrv,
    }
    source._cmd_pub = FakePublisher()
    source._led_pub = FakePublisher()
    source._clients = {
        n: n for n in ('power_on', 'release_servos', 'set_gripper', 'set_read_only')
    }
    feeders = []

    def feed(angles):
        f = Feeder(source, angles)
        feeders.append(f)
        time.sleep(0.05)
        return f

    source.feed = feed
    source.bridge = bridge
    yield source
    source.close()
    for f in feeders:
        f.close()


def wait_for(predicate, timeout=3.0):
    end = time.time() + timeout
    while time.time() < end:
        if predicate():
            return True
        time.sleep(0.01)
    return False


def moves(source):
    return [m for m in source._cmd_pub.sent if m.type == FakeArmCommand.ALL_ANGLES]


# ---- telemetry ----------------------------------------------------------------------


def test_joint_states_become_degrees_in_joint_order(ros_arm):
    names = ['joint6', 'joint5', 'joint4', 'joint3', 'joint2', 'joint1']
    ros_arm._on_joint_state(joint_state([6, 5, 4, 3, 2, 1], names))
    s = ros_arm.snapshot()
    assert s['source'] == 'ros' and s['connected'] is True and s['error'] is None
    assert s['angles'] == [1, 2, 3, 4, 5, 6]
    assert len(s['coords']) == 6
    assert 'gripper' in s['capabilities'] and 'rest' in s['capabilities']


def test_temperatures_and_rest_mode(ros_arm):
    ros_arm._on_temps(FakeMsg(celsius=[30, 56, 66, 40, 41, 42], valid=True))
    s = ros_arm.snapshot()
    assert s['temps'] == [30, 56, 66, 40, 41, 42]
    assert s['temp_levels'] == ['normal', 'warm', 'hot', 'normal', 'normal', 'normal']
    ros_arm._on_temps(FakeMsg(celsius=[0] * 6, valid=False))
    assert ros_arm.snapshot()['temps'] is None
    ros_arm._on_read_only(FakeMsg(data=True))
    assert ros_arm.snapshot()['resting'] is True


def test_stale_joint_states_disconnect(ros_arm, monkeypatch):
    monkeypatch.setattr(arm, 'ROS_STALE_AFTER', 0.1)
    ros_arm._on_joint_state(joint_state([0] * 6))
    ros_arm.start()
    assert wait_for(lambda: ros_arm.snapshot()['connected'] is False)
    assert 'arm_driver' in ros_arm.snapshot()['error']


def test_commands_before_ros_is_ready():
    source = arm.RosArm(bridge=FakeBridge())
    with pytest.raises(RuntimeError, match='fake bridge'):
        source.stop_motion()


def test_no_move_without_fresh_feedback(ros_arm):
    with pytest.raises(RuntimeError, match='joint_states'):
        ros_arm.move_joint(0, 10)


# ---- streamed moves -------------------------------------------------------------------


def test_move_is_streamed_in_bounded_steps(ros_arm):
    feeder = ros_arm.feed([0] * 6)  # the arm does not follow: the stream keeps going
    ros_arm.move_joint(0, 60, speed=40)
    assert wait_for(lambda: len(moves(ros_arm)) >= 4)
    sent = moves(ros_arm)
    for m in sent:
        assert m.angles_deg == [15.0, 0, 0, 0, 0, 0]  # at most ROS_STEP_DEG ahead
        assert m.speed == 40
    gaps = [b - a for a, b in zip(ros_arm._cmd_pub.times, ros_arm._cmd_pub.times[1:])]
    assert max(gaps) < 0.3  # always inside arm_driver's 300 ms watchdog

    feeder.angles = [60, 0, 0, 0, 0, 0]  # arrived
    time.sleep(0.2)
    count = len(moves(ros_arm))
    time.sleep(0.3)
    assert len(moves(ros_arm)) == count  # the stream ended by itself


def test_all_joints_arrive_together(ros_arm):
    ros_arm.feed([0, 0, 0, 0, 0, 0])
    ros_arm._start_move([30, -60, 0, 0, 0, 0], 30)
    assert wait_for(lambda: moves(ros_arm))
    assert moves(ros_arm)[0].angles_deg == [7.5, -15.0, 0, 0, 0, 0]


def test_stop_cancels_the_stream(ros_arm):
    ros_arm.feed([0] * 6)
    ros_arm.move_joint(2, 90)
    assert wait_for(lambda: moves(ros_arm))
    ros_arm.stop_motion()
    assert ros_arm._cmd_pub.sent[-1].type == FakeArmCommand.STOP
    count = len(ros_arm._cmd_pub.sent)
    time.sleep(0.3)
    assert len(ros_arm._cmd_pub.sent) == count


def test_newer_command_replaces_the_running_one(ros_arm):
    ros_arm.feed([0] * 6)
    ros_arm.move_joint(0, 90)
    assert wait_for(lambda: moves(ros_arm))
    ros_arm.move_joint(1, -90)
    time.sleep(0.25)
    assert moves(ros_arm)[-1].angles_deg == [0, -15.0, 0, 0, 0, 0]


def test_move_timeout_stops_and_notifies(ros_arm, monkeypatch):
    monkeypatch.setattr(arm, 'ROS_MOVE_TIMEOUT', 0.3)
    ros_arm.feed([0] * 6)
    ros_arm.move_joint(0, 90)
    assert wait_for(lambda: ros_arm.snapshot()['notice'] is not None)
    assert ros_arm._cmd_pub.sent[-1].type == FakeArmCommand.STOP
    assert 'not reached' in ros_arm.snapshot()['notice']['text']


# ---- poses, rest, wake ------------------------------------------------------------------


def test_park_keeps_the_servos_on(ros_arm):
    feeder = ros_arm.feed([0] * 6)
    ros_arm.go_to_pose('park')
    assert wait_for(lambda: moves(ros_arm))
    feeder.angles = list(arm.POSES['park'])
    time.sleep(0.4)
    assert ros_arm.bridge.calls == []  # no release, no rest mode
    assert ros_arm.snapshot()['resting'] is False


def test_zero_streams_all_angles(ros_arm):
    ros_arm.feed([10] * 6)
    ros_arm.go_to_pose('zero')
    assert wait_for(lambda: moves(ros_arm))
    assert moves(ros_arm)[0].type == FakeArmCommand.ALL_ANGLES


def test_resting_arm_refuses_motion(ros_arm):
    ros_arm.feed([0] * 6)
    ros_arm._on_read_only(FakeMsg(data=True))
    with pytest.raises(arm.RestingError):
        ros_arm.move_joint(0, 10)
    with pytest.raises(arm.RestingError):
        ros_arm.go_to_pose('zero')
    with pytest.raises(arm.RestingError):
        ros_arm.set_gripper(True)
    ros_arm.stop_motion()  # STOP always goes out


def test_wake_leaves_rest_and_powers_on_without_moving(ros_arm):
    ros_arm.feed([0, 131, -148, 146, 4.5, 0])
    ros_arm._on_read_only(FakeMsg(data=True))
    ros_arm.wake_up()
    assert ros_arm.bridge.calls == [('set_read_only', False), ('power_on', None)]
    s = ros_arm.snapshot()
    assert s['resting'] is False and s['power_on'] is True
    time.sleep(0.3)
    assert moves(ros_arm) == []  # it stays in the park pose


def test_service_refusal_is_an_error(ros_arm):
    ros_arm.feed([0] * 6)
    ros_arm.bridge.fail.add('power_on')
    with pytest.raises(RuntimeError, match='power_on refused'):
        ros_arm.wake_up()


def test_gripper_led_and_rest(ros_arm):
    ros_arm.feed([0] * 6)
    ros_arm.set_gripper(True)
    assert ros_arm.bridge.calls[-1] == ('set_gripper', True)
    assert ros_arm.snapshot()['gripper'] == 'closed'
    ros_arm.set_led(255, 0, 51)
    sent = ros_arm._led_pub.sent[-1]
    assert (sent.r, sent.g, round(sent.b, 1)) == (1.0, 0.0, 0.2)
    assert ros_arm.snapshot()['led'] == [255, 0, 51]
    ros_arm.set_rest(True)
    assert ros_arm.bridge.calls[-1] == ('set_read_only', True)
    assert ros_arm.snapshot()['resting'] is True


def test_park_pose_matches_park_py():
    assert arm.POSES['park'] == [0, 131, -148, 146, 4.5, 0]


# ---- mock arm extras ----------------------------------------------------------------------


def test_mock_rest_gripper_led():
    mock = arm.MockArm()
    mock.set_led(1, 2, 3)
    mock.set_gripper(False)
    mock.set_rest(True)
    s = mock.snapshot()
    assert s['led'] == [1, 2, 3] and s['gripper'] == 'open' and s['resting'] is True
    with pytest.raises(arm.RestingError):
        mock.set_gripper(True)
    mock.wake_up()
    s = mock.snapshot()
    assert s['resting'] is False and s['power_on'] is True


def test_mock_park_does_not_rest():
    mock = arm.MockArm()
    mock.go_to_pose('park')
    time.sleep(0.1)
    s = mock.snapshot()
    assert s['resting'] is False and s['power_on'] is True


# ---- ROS panels without ROS ----------------------------------------------------------------


def test_panels_report_unavailable_ros():
    bridge = ros.RosBridge()
    bridge.error = 'rclpy not available'
    for panel in (ros_health.Health(bridge), ros_graph.Graph(bridge)):
        assert panel.snapshot() == {'available': False, 'error': 'rclpy not available'}
    assert ros_graph.Graph(bridge).watch('/chatter', 'a') == 'rclpy not available'


def test_bridge_without_rclpy(monkeypatch):
    import sys

    monkeypatch.setitem(sys.modules, 'rclpy', None)
    bridge = ros.RosBridge()
    seen = []
    bridge.when_ready(seen.append)
    bridge.start()
    assert wait_for(lambda: seen == [None])
    assert 'rclpy not available' in bridge.error and bridge.available is False
    bridge.when_ready(seen.append)  # after the fact: called at once
    assert seen == [None, None]


def test_diagnostic_levels():
    assert ros_health.level_name(b'\x00') == 'ok'
    assert ros_health.level_name(b'\x01') == 'warn'
    assert ros_health.level_name(2) == 'error'
    assert ros_health.level_name(b'\x09') == 'unknown'


def test_echo_payload_is_plain_and_bounded():
    import array

    data = ros_graph.jsonable(
        {'a': array.array('d', [1.0, 2.0]), 'b': b'\x01\x02', 'c': (1, 'x'), 'd': object}
    )
    assert (
        data['a'] == [1.0, 2.0]
        and data['b'] == [1, 2]
        and data['c'] == [1, 'x']
        and isinstance(data['d'], str)
    )
    assert ros_graph.cut({'x': 1}) == {'x': 1}
    long = ros_graph.cut({'x': 'y' * 10000})
    assert (
        isinstance(long, str)
        and long.endswith('(cut)')
        and len(long) < ros_graph.ECHO_MAX_CHARS + 20
    )


def test_internal_services_are_flagged():
    assert ros_graph.is_internal_service('/arm_driver/get_parameters')
    assert ros_graph.is_internal_service('/fibonacci/_action/send_goal')
    assert not ros_graph.is_internal_service('/arm/power_on')


def test_echo_rate():
    echo = ros_graph._Echo('/t', 'std_msgs/msg/String')
    now = time.time()
    for i in range(11):
        echo.stamps.append(now - 1 + i * 0.1)
    assert echo.rate(now) == 10.0
