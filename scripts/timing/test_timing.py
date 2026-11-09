"""Unit tests of the timing scripts: reply framing, statistics, and the STOP handling.

make timing-test (part of make ci). No serial port and no ROS: the ROS modules that hold_load
imports are replaced by small fakes, so its STOP logic runs on the host.
"""

import importlib
import sys
import types

import pytest
import serial_rtt
from stats import interval_stats, percentile

FRAME = serial_rtt.REPLY_HEAD + bytes(range(12)) + b'\xfa'  # a complete getAngles reply


class FakePort:
    """Gives the bytes in `chunks`, one chunk per read(); then nothing."""

    def __init__(self, *chunks):
        self.chunks = [bytes(c) for c in chunks]

    @property
    def in_waiting(self):
        return len(self.chunks[0]) if self.chunks else 0

    def read(self, size):
        return self.chunks.pop(0) if self.chunks else b''


@pytest.fixture(autouse=True)
def short_timeout(monkeypatch):
    monkeypatch.setattr(serial_rtt, 'REPLY_TIMEOUT_S', 0.05)


def test_frame_is_17_bytes():
    assert serial_rtt.REPLY_LEN == len(FRAME) == 17


def test_complete_reply():
    ok, raw = serial_rtt.read_reply(FakePort(FRAME))
    assert ok and raw == FRAME


def test_reply_split_over_reads():
    ok, _ = serial_rtt.read_reply(FakePort(FRAME[:5], FRAME[5:11], FRAME[11:]))
    assert ok


def test_noise_before_the_reply():
    ok, _ = serial_rtt.read_reply(FakePort(b'\x00\xfe\x13', FRAME))
    assert ok


def test_one_byte_short_times_out():
    ok, raw = serial_rtt.read_reply(FakePort(FRAME[:-1]))
    assert not ok and raw == FRAME[:-1]


def test_wrong_footer_then_a_good_reply():
    bad = FRAME[:-1] + b'\x00'
    ok, _ = serial_rtt.read_reply(FakePort(bad, FRAME))
    assert ok


def test_wrong_footer_alone_times_out():
    ok, _ = serial_rtt.read_reply(FakePort(FRAME[:-1] + b'\x00'))
    assert not ok


def test_nothing_times_out():
    ok, raw = serial_rtt.read_reply(FakePort())
    assert not ok and raw == b''


@pytest.mark.parametrize('p, expected', [(0.0, 1), (0.01, 1), (0.5, 50), (0.99, 99), (1.0, 100)])
def test_percentile_nearest_rank(p, expected):
    assert percentile(list(range(1, 101)), p) == expected


def test_percentile_one_value():
    assert percentile([7.5], 0.99) == 7.5


def test_percentile_empty():
    with pytest.raises(ValueError):
        percentile([], 0.5)


def test_interval_stats_steady_rate():
    s = interval_stats([0.0, 0.05, 0.10, 0.15, 0.20], 20)
    assert s['achieved_hz'] == pytest.approx(20.0)
    assert s['mean_ms'] == pytest.approx(50.0)
    assert s['max_ms'] == pytest.approx(50.0)
    assert s['late_pct'] == 0.0


def test_interval_stats_counts_late_intervals():
    s = interval_stats([0.0, 0.05, 0.20], 20)  # 50 ms, then 150 ms (> 1.5 x 50 ms)
    assert s['late_pct'] == pytest.approx(50.0)
    assert s['max_ms'] == pytest.approx(150.0)
    assert s['achieved_hz'] == pytest.approx(10.0)


@pytest.mark.parametrize('stamps', [[], [1.0], [1.0, 1.0]])
def test_interval_stats_needs_advancing_stamps(stamps):
    with pytest.raises(ValueError):
        interval_stats(stamps, 20)


# ---- hold_load's STOP handling, with fake ROS modules -----------------------------------------

STOP, ALL_ANGLES = 0, 2


class FakeMsg:
    STOP, JOINTS, ALL_ANGLES = STOP, 1, ALL_ANGLES

    def __init__(self, **fields):
        self.__dict__.update(fields)


class FakeNode:
    def __init__(self):
        self.subscriptions = []

    def create_subscription(self, msg_type, topic, callback, qos):
        self.subscriptions.append((topic, callback))


class FakePublisher:
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


@pytest.fixture
def hold_load(monkeypatch):
    """hold_load imported against fake rclpy, braco_interfaces and sensor_msgs."""
    fakes = {
        'rclpy': types.ModuleType('rclpy'),
        'rclpy.node': types.ModuleType('rclpy.node'),
        'braco_interfaces': types.ModuleType('braco_interfaces'),
        'braco_interfaces.msg': types.ModuleType('braco_interfaces.msg'),
        'braco_interfaces.srv': types.ModuleType('braco_interfaces.srv'),
        'sensor_msgs': types.ModuleType('sensor_msgs'),
        'sensor_msgs.msg': types.ModuleType('sensor_msgs.msg'),
    }
    fakes['rclpy.node'].Node = FakeNode
    fakes['braco_interfaces.msg'].ArmCommand = FakeMsg
    fakes['braco_interfaces.srv'].TakeControl = FakeMsg
    fakes['sensor_msgs.msg'].JointState = FakeMsg
    for name, module in fakes.items():
        monkeypatch.setitem(sys.modules, name, module)
    monkeypatch.delitem(sys.modules, 'hold_load', raising=False)
    module = importlib.import_module('hold_load')
    yield module
    sys.modules.pop('hold_load', None)


def test_watch_stop_is_set_by_a_stop_only(hold_load):
    node = FakeNode()
    stop = hold_load.watch_stop(node)
    [(topic, on_command)] = node.subscriptions
    assert topic == 'arm/command'
    on_command(FakeMsg(type=ALL_ANGLES))
    assert stop['stop'] is False
    on_command(FakeMsg(type=STOP))
    assert stop['stop'] is True


def test_nothing_is_published_after_a_stop(hold_load):
    pub, stop, cmd = FakePublisher(), {'stop': False}, FakeMsg(type=ALL_ANGLES)
    assert hold_load.publish_unless_stopped(pub, cmd, stop) is True
    stop['stop'] = True
    assert hold_load.publish_unless_stopped(pub, cmd, stop) is False
    assert pub.sent == [cmd]


def test_release_control_never_raises(hold_load, monkeypatch, capsys):
    def gone(node, release=False):
        raise RuntimeError('arm/take_control not available')

    monkeypatch.setattr(hold_load, 'take_control', gone)
    hold_load.release_control(FakeNode())
    assert 'could not release the control' in capsys.readouterr().out
