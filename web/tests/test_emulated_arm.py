"""ArmSource with the REAL pymycobot against an emulated arm on a pseudo-terminal.

The emulator answers the serial frames (0xFE 0xFE <len> <cmd> <data> 0xFA) the
way the myCobot does and records every command byte it receives, so these tests
prove which commands the dashboard actually puts on the wire.

Skipped when pymycobot is not installed or the OS has no pseudo-terminals.
"""

import os
import struct
import threading
import time

import pytest

pytest.importorskip("pymycobot")
tty = pytest.importorskip("tty")

from web.server import arm  # noqa: E402

READ_COMMANDS = {0x20, 0x23, 0x12, 0x2B, 0xE5}   # GET_ANGLES, GET_COORDS, IS_POWER_ON, IS_MOVING, GET_SERVO_TEMPS
STOP = 0x29
# POWER_ON/OFF, RELEASE_ALL_SERVOS, SEND_ANGLE(S), SET_SERVO_DATA, SET_GRIPPER_STATE, SET_COLOR
MOTION_OR_WRITE = {0x10, 0x11, 0x13, 0x21, 0x22, 0x52, 0x66, 0x6A}

ANGLES = [10.0, -20.0, 30.0, -40.0, 50.0, -60.0]
TEMPS = [30, 56, 66, 40, 41, 42]


def i16(value):
    return list(struct.pack(">h", int(round(value))))


class EmulatedArm:
    def __init__(self):
        self.master, self.slave = os.openpty()
        tty.setraw(self.master)
        tty.setraw(self.slave)
        self.port = os.ttyname(self.slave)
        self.received = []
        self.silent = False
        self.ignore_reads = False   # answer only STOP
        threading.Thread(target=self._serve, daemon=True).start()

    def _reply(self, cmd, data):
        os.write(self.master, bytes([0xFE, 0xFE, len(data) + 2, cmd] + data + [0xFA]))

    def _answer(self, cmd):
        if self.silent or (self.ignore_reads and cmd != STOP):
            return
        if cmd == 0x20:
            self._reply(cmd, sum((i16(a * 100) for a in ANGLES), []))
        elif cmd == 0x23:
            self._reply(cmd, sum((i16(v * 10) for v in (100, 0, 200)), [])
                        + sum((i16(v * 100) for v in (1, 2, 3)), []))
        elif cmd == 0x12:
            self._reply(cmd, [1])
        elif cmd == 0x2B:
            self._reply(cmd, [0])
        elif cmd == 0xE5:
            self._reply(cmd, list(TEMPS))
        elif cmd == STOP:
            # pymycobot 4.x waits for an acknowledgement and re-sends STOP
            # up to 3 times (~0.5 s each) when none arrives
            self._reply(cmd, [1])

    def _serve(self):
        buf = b""
        while True:
            try:
                buf += os.read(self.master, 256)
            except OSError:
                return   # closed at teardown
            while True:
                start = buf.find(b"\xfe\xfe")
                if start < 0 or len(buf) < start + 4:
                    break
                end = start + 3 + buf[start + 2]
                if len(buf) < end:
                    break
                cmd = buf[start + 3]
                buf = buf[end:]
                self.received.append(cmd)
                self._answer(cmd)

    def close(self):
        for fd in (self.master, self.slave):
            try:
                os.close(fd)
            except OSError:
                pass


def wait_for(predicate, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        if predicate():
            return True
        time.sleep(0.02)
    return False


@pytest.fixture
def emulated(monkeypatch):
    monkeypatch.setattr(arm, "ANGLE_PERIOD", 0.02)
    emu = EmulatedArm()
    yield emu
    emu.close()


@pytest.fixture
def source(emulated):
    """ArmSource on the emulator, closed (thread joined, port closed) after the test."""
    src = arm.ArmSource(emulated.port, 1000000)
    src.start()
    yield src
    src.close()


def test_reads_real_values_through_pymycobot(source):
    assert wait_for(lambda: source.snapshot()["temps"] is not None)

    s = source.snapshot()
    assert s["connected"] is True and s["error"] is None
    assert s["angles"] == ANGLES
    assert s["coords"] == [100.0, 0.0, 200.0, 1.0, 2.0, 3.0]
    assert s["power_on"] is True and s["moving"] is False
    assert s["temps"] == TEMPS
    assert s["temp_levels"] == ["normal", "warm", "hot", "normal", "normal", "normal"]


def test_only_read_commands_on_the_wire(emulated, source):
    assert wait_for(lambda: READ_COMMANDS <= set(emulated.received))
    time.sleep(0.5)

    sent = set(emulated.received)
    assert sent == READ_COMMANDS, "unexpected commands: {}".format(sorted(map(hex, sent - READ_COMMANDS)))
    assert not sent & MOTION_OR_WRITE


def test_stop_sends_only_stop(emulated, source):
    assert wait_for(lambda: source.snapshot()["connected"])
    before = len(emulated.received)

    source.stop_motion()
    assert wait_for(lambda: STOP in emulated.received[before:])
    assert not set(emulated.received) & MOTION_OR_WRITE


def test_arm_stops_answering(emulated, source):
    assert wait_for(lambda: source.snapshot()["connected"])

    emulated.silent = True
    start = time.time()
    assert wait_for(lambda: not source.snapshot()["connected"], timeout=10)
    assert time.time() - start < 5, "took {:.1f} s to notice".format(time.time() - start)
    s = source.snapshot()
    assert s["error"] == "no reply from the arm"
    assert s["angles"] is None   # never show stale angles as live

    emulated.silent = False
    assert wait_for(lambda: source.snapshot()["connected"], timeout=10)
    assert source.snapshot()["angles"] == ANGLES


def test_stop_is_not_blocked_by_polling(emulated, source):
    """While the arm does not answer reads, each read waits for pymycobot's
    timeout. STOP must wait for at most the one read in progress, not for the
    whole polling cycle."""
    assert wait_for(lambda: source.snapshot()["connected"])
    emulated.silent = True
    time.sleep(0.3)   # the poller is now inside a read that will time out
    emulated.silent = False
    emulated.ignore_reads = True

    start = time.time()
    source.stop_motion()
    elapsed = time.time() - start
    assert STOP in emulated.received
    assert elapsed < 2.0, "STOP waited {:.1f} s".format(elapsed)


def test_close_stops_polling(emulated, source):
    assert wait_for(lambda: source.snapshot()["connected"])
    source.close()
    count = len(emulated.received)
    time.sleep(0.3)
    assert len(emulated.received) == count
