"""Unit tests of the timing scripts' pure parts: the getAngles reply framing and the percentile.

make timing-test (part of make ci). No serial port, no ROS.
"""

import pytest
import serial_rtt
from stats import percentile

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
