"""Read-only telemetry from the myCobot 280 for the web dashboard.

ArmSource talks to the real arm through pymycobot; MockArm produces smooth
synthetic data so the page can be developed on a PC without the arm. Both
expose the same interface: start(), snapshot() and stop_motion().

Besides reading, the only commands sent to the arm are stop() and a move to
one of the fixed POSES (park / zero), both started by an explicit button.
Nothing here powers the servos or writes a setting.
"""

import math
import random
import threading
import time

DEFAULT_PORT = "/dev/ttyTHS1"
DEFAULT_BAUD = 1000000

# Same thresholds as TEMP_WARM_C / TEMP_HOT_C in libs/cpp/includes/mycobot.hpp
TEMP_WARM_C = 55
TEMP_HOT_C = 65

# DEFAULT_SOFT_LIMITS from mycobot.hpp, used by the mock
SOFT_LIMITS = [(-160, 160), (-130, 130), (-140, 140), (-120, 120), (-135, 135), (-170, 170)]

# myCobot 280 standard DH: theta offset (rad), d (mm), a (mm), alpha (rad).
# Same table as web/static/js/arm3d.js.
DH = [
    (0, 131.22, 0, math.pi / 2),
    (-math.pi / 2, 0, -110.4, 0),
    (0, 0, -96, 0),
    (-math.pi / 2, 63.4, 0, math.pi / 2),
    (math.pi / 2, 75.05, 0, -math.pi / 2),
    (0, 45.6, 0, 0),
]

# Fixed poses, same as playground/park.py and playground/zero.py
POSES = {
    "park": [0, 131, -150, 146, 4.5, 0],
    "zero": [0, 0, 0, 0, 0, 0],
}
POSE_SPEED = 30
RELEASE_POSES = {"park"}   # servos are released (arm goes limp) once the pose is reached
RELEASE_TIMEOUT = 20       # give up waiting for the arm to arrive after this many seconds
POSE_HOLD = 10   # mock only: seconds to stay in the pose before wandering again

ANGLE_PERIOD = 0.2   # 5 Hz
STATE_PERIOD = 1.0   # power / moving
TEMP_PERIOD = 2.0    # same as temps_lcd.cpp: temperatures change slowly


class RestingError(RuntimeError):
    """The arm is resting (servos released): wake_up() first."""


def check_awake(resting):
    if resting:
        raise RestingError("arm is resting, wake it up first")


def temp_level(celsius):
    if celsius is None:
        return None
    if celsius >= TEMP_HOT_C:
        return "hot"
    if celsius >= TEMP_WARM_C:
        return "warm"
    return "normal"


def read_value(getter, retries=5):
    """Call getter() until it returns a list of 6 numbers, or give up.

    Same pattern as playground/get_angles.py: pymycobot sometimes returns
    -1 or None when a reply is missed.
    """
    for _ in range(retries):
        value = getter()
        if isinstance(value, list) and len(value) == 6:
            return value
        time.sleep(0.05)
    return None


def as_bool(value):
    """pymycobot returns 1 / 0 / -1 for yes / no / no reply."""
    if value in (0, 1):
        return bool(value)
    return None


class _Base:
    def __init__(self):
        self._lock = threading.Lock()
        self._closed = threading.Event()
        self._state = {
            "mock": False,
            "connected": False,
            "error": None,
            "angles": None,
            "coords": None,
            "temps": None,
            "temp_levels": None,
            "power_on": None,
            "resting": False,   # sticky: set when a park finishes, cleared by wake_up()
            "moving": None,
            "updated": None,
        }

    def snapshot(self):
        with self._lock:
            return dict(self._state)

    def _update(self, **fields):
        with self._lock:
            self._state.update(fields)
            self._state["updated"] = time.time()

    def close(self):
        """Stop the polling thread (server shutdown, tests)."""
        self._closed.set()

    def _set_temps(self, temps):
        levels = [temp_level(t) for t in temps] if temps else None
        self._update(temps=temps, temp_levels=levels)


class ArmSource(_Base):
    def __init__(self, port=DEFAULT_PORT, baud=DEFAULT_BAUD):
        super().__init__()
        self.port = port
        self.baud = baud
        self._mc = None
        self._motion = 0   # bumped by every move command
        self._serial = threading.Lock()   # pymycobot is not thread-safe

    def start(self):
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def close(self):
        super().close()
        thread = getattr(self, "_thread", None)
        if thread is not None:
            thread.join(timeout=5)

    def _connect(self):
        # Newer pymycobot versions provide MyCobot280; older ones only have MyCobot.
        try:
            from pymycobot import MyCobot280 as MyCobot
        except ImportError:
            from pymycobot.mycobot import MyCobot
        self._mc = MyCobot(self.port, self.baud)
        time.sleep(0.5)

    def _run(self):
        last_state = last_temp = 0.0
        while not self._closed.is_set():
            if self._mc is None:
                try:
                    self._connect()
                    self._update(connected=True, error=None)
                except Exception as e:   # port busy, missing, no permission
                    self._update(connected=False, error=str(e))
                    self._closed.wait(3)
                    continue

            try:
                # A missed reply costs pymycobot ~1.5 s, so keep retries low and
                # skip the other reads while the arm is not answering.
                angles = read_value(lambda: self._call(self._mc.get_angles), retries=2)
                if angles is None:
                    self._update(angles=None, coords=None, connected=False,
                                 error="no reply from the arm")
                    self._closed.wait(ANGLE_PERIOD)
                    continue
                coords = read_value(lambda: self._call(self._mc.get_coords), retries=1)
                self._update(angles=angles, coords=coords, connected=True, error=None)

                now = time.time()
                if now - last_state >= STATE_PERIOD:
                    last_state = now
                    power = as_bool(self._call(self._mc.is_power_on))
                    moving = as_bool(self._call(self._mc.is_moving))
                    self._update(power_on=power, moving=moving)

                if now - last_temp >= TEMP_PERIOD:
                    last_temp = now
                    self._set_temps(self._read_temps())
            except Exception as e:   # serial unplugged mid-run
                self._update(connected=False, error=str(e))
                self._mc = None
                self._closed.wait(1)
                continue

            self._closed.wait(ANGLE_PERIOD)
        self._close_port()

    def _close_port(self):
        port = getattr(self._mc, "_serial_port", None)
        if port is not None:
            with self._serial:
                port.close()
        self._mc = None

    def _call(self, fn):
        """One serial request. The lock is held per request, so stop_motion()
        never waits for more than the request in progress."""
        with self._serial:
            return fn()

    def _read_temps(self):
        getter = getattr(self._mc, "get_servo_temps", None)
        if getter is None:
            return None
        value = self._call(getter)
        if isinstance(value, list) and len(value) >= 6:
            return [int(round(t)) for t in value[:6]]
        return None

    def stop_motion(self):
        if self._mc is None:
            raise RuntimeError("arm not connected")
        self._motion += 1   # a STOP also cancels a pending release
        with self._serial:
            self._mc.stop()

    def go_to_pose(self, name, speed=POSE_SPEED):
        if name not in POSES:
            raise ValueError("unknown pose: " + name)
        if self._mc is None:
            raise RuntimeError("arm not connected")
        check_awake(self.snapshot()["resting"])
        self._motion += 1   # cancels a release still waiting from an earlier park
        with self._serial:
            self._mc.send_angles(POSES[name], speed)
        if name in RELEASE_POSES:
            threading.Thread(target=self._release_when_arrived,
                             args=(self._motion,), daemon=True).start()

    def move_joint(self, index, angle, speed=POSE_SPEED):
        """Move one joint (0-based index); the others are not commanded."""
        if self._mc is None:
            raise RuntimeError("arm not connected")
        check_awake(self.snapshot()["resting"])
        self._motion += 1
        with self._serial:
            self._mc.send_angle(index + 1, angle, speed)

    def wake_up(self, speed=POSE_SPEED):
        """Power the servos back on after a rest and move to the zero pose."""
        if self._mc is None:
            raise RuntimeError("arm not connected")
        self._motion += 1
        with self._serial:
            self._mc.power_on()
        time.sleep(0.3)
        self._update(power_on=True, resting=False)
        with self._serial:
            self._mc.send_angles(POSES["zero"], speed)

    def _release_when_arrived(self, motion):
        """Wait for the arm to stop moving, then release the servos."""
        deadline = time.time() + RELEASE_TIMEOUT
        time.sleep(1.0)   # is_moving only turns true a moment after the command
        while time.time() < deadline and motion == self._motion and not self._closed.is_set():
            try:
                if self._call(self._mc.is_moving) == 0:
                    break
            except Exception:
                return
            time.sleep(0.3)
        if motion != self._motion or self._closed.is_set():
            return   # another command or STOP took over
        try:
            self._call(self._mc.release_all_servos)
            self._update(power_on=False, resting=True)
        except Exception:
            pass


class MockArm(_Base):
    """Synthetic arm: slow sine waves inside the soft limits, warming servos."""

    def __init__(self):
        super().__init__()
        self._t0 = time.time()
        self._temps = [32.0, 35.0, 36.0, 38.0, 34.0, 31.0]
        self._stopped_until = 0.0
        self._frozen = None
        self._release_at = None
        self._pose = None   # (angles, started_at, from_angles) while holding a pose
        self._update(mock=True, connected=True, power_on=True)

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        last_temp = 0.0
        while not self._closed.is_set():
            now = time.time()
            if self._release_at and now >= self._release_at:
                self._release_at = None
                self._update(power_on=False, resting=True)
            if self.snapshot()["resting"]:
                angles, moving = self._frozen, False   # resting: servos released
            elif now < self._stopped_until:
                angles, moving = self._frozen, False
            elif self._pose and now < self._pose[1] + 3 + POSE_HOLD:
                target, started, origin = self._pose
                k = min(1.0, (now - started) / 3)
                angles = [round(o + (g - o) * k, 2) for o, g in zip(origin, target)]
                moving = k < 1.0
            else:
                t = now - self._t0
                angles = [
                    round(0.55 * hi * math.sin(t * (0.15 + 0.05 * i) + i), 2)
                    for i, (_, hi) in enumerate(SOFT_LIMITS)
                ]
                moving = True
            self._frozen = angles
            coords = self._fake_coords(angles)
            self._update(angles=angles, coords=coords, moving=moving)

            if now - last_temp >= TEMP_PERIOD:
                last_temp = now
                for i in range(6):
                    drift = random.uniform(-0.3, 0.9) if moving else random.uniform(-0.8, 0.1)
                    self._temps[i] = min(72.0, max(28.0, self._temps[i] + drift))
                self._set_temps([int(round(t)) for t in self._temps])

            self._closed.wait(ANGLE_PERIOD)

    @staticmethod
    def _fake_coords(angles):
        """Flange position from the arm's DH parameters (same as the 3D view)."""
        m = [[1.0, 0, 0, 0], [0, 1.0, 0, 0], [0, 0, 1.0, 0], [0, 0, 0, 1.0]]
        for (offset, d, a, alpha), q in zip(DH, angles):
            th = math.radians(q) + offset
            c, s, ca, sa = math.cos(th), math.sin(th), math.cos(alpha), math.sin(alpha)
            t = [[c, -s * ca, s * sa, a * c], [s, c * ca, -c * sa, a * s],
                 [0, sa, ca, d], [0, 0, 0, 1]]
            m = [[sum(m[i][k] * t[k][j] for k in range(4)) for j in range(4)] for i in range(4)]
        return [round(m[0][3], 1), round(m[1][3], 1), round(m[2][3], 1),
                round(angles[3], 2), round(angles[4], 2), round(angles[5], 2)]

    def stop_motion(self):
        self._release_at = None
        self._pose = None
        self._stopped_until = time.time() + 5

    def go_to_pose(self, name, speed=POSE_SPEED):
        if name not in POSES:
            raise ValueError("unknown pose: " + name)
        check_awake(self.snapshot()["resting"])
        self._start_move(POSES[name], release=name in RELEASE_POSES)

    def move_joint(self, index, angle, speed=POSE_SPEED):
        check_awake(self.snapshot()["resting"])
        target = list(self._frozen or [0.0] * 6)
        target[index] = angle
        self._start_move(target)

    def wake_up(self, speed=POSE_SPEED):
        self._start_move(POSES["zero"])   # powers on, then goes to zero

    def _start_move(self, target, release=False):
        self._update(power_on=True, resting=False)
        self._release_at = time.time() + 3 if release else None
        origin = list(self._frozen or [0.0] * 6)
        self._stopped_until = 0.0
        self._pose = (target, time.time(), origin)
