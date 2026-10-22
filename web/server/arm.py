"""Arm sources for the braco web page: telemetry plus a small set of commands.

    RosArm     the real arm through arm_driver (ROS 2), the only way to the real arm
    GazeboArm  the arm simulated in Gazebo (ROS 2, PC only)
    MockArm    synthetic data, for developing the page without the arm

The page never opens the serial port: arm_driver owns it (docs/architecture.html).

All of them expose start(), snapshot(), stop_motion(), go_to_pose(), move_joint()
and wake_up() (leave rest, servos on, no movement); RosArm and MockArm also set_rest(), set_gripper() and set_led().
take_control() / release_control() follow the page's control lease: RosArm asks arm_driver
for the control (docs/ADR-0002-controle-unico.md), the others have nobody to ask.
snapshot()["capabilities"] tells the page which of these a source supports.
Commands only come from explicit actions on the page; nothing here powers the
servos or moves the arm by itself, and no setting is written.
"""

import math
import random
import threading
import time

from . import ros
from .ros_health import level_name
from .session import ControlError

# Same thresholds as TEMP_WARM_C / TEMP_HOT_C in ros2_ws/src/mycobot_cpp/includes/mycobot.hpp. Only the mock
# uses them: on the real arm the level comes from temperature_monitor (/diagnostics).
TEMP_WARM_C = 55
TEMP_HOT_C = 65

# JOINT_LIMITS from mycobot.hpp, used by the mock
JOINT_LIMITS = [(-166, 166), (-138, 138), (-149, 149), (-149, 149), (-153, 158), (-178, 178)]

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

# Fixed poses, same as ZERO_ANGLES / PARK_ANGLES in ros2_ws/src/mycobot_cpp/includes/mycobot.hpp
# (J3 -148: just inside the J3 limit)
POSES = {
    "park": [0, 131, -148, 146, 4.5, 0],
    "zero": [0, 0, 0, 0, 0, 0],
}
POSE_SPEED = 30
POSE_HOLD = 10   # mock only: seconds to stay in the pose before wandering again

# Gazebo (ros2_control) source. Joint names follow the myCobot 280 URDF; override
# them with GZ_JOINT_NAMES (comma separated) if the simulated model differs.
GZ_JOINT_NAMES = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
                  "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]
GZ_STATE_TOPIC = "/joint_states"
GZ_TRAJ_TOPIC = "/joint_trajectory_controller/joint_trajectory"
GZ_DEG_PER_SPEED = 1.0   # the arm's speed (1-100) in degrees per second
GZ_MIN_MOVE_TIME = 0.5   # seconds, so a tiny move is not an instant jump
GZ_MOVING_EPS = 0.05     # degrees: smaller changes between two states count as standing still
GZ_STALE_AFTER = 2.0     # seconds without /joint_states before the page shows "disconnected"

# arm_driver (ROS 2) source: same values as ros2_ws/src/braco_bringup/src/arm_moves.cpp.
# The driver stops the arm when no motion command arrives for 300 ms, so a move is a
# stream of targets at most ROS_STEP_DEG ahead of the real angles, resent every ROS_RESEND_S.
ROS_STEP_DEG = 15.0
ROS_RESEND_S = 0.1
ROS_TOLERANCE_DEG = 2.0
ROS_MOVE_TIMEOUT = 20.0
ROS_PARK_SETTLE_S = 0.5   # same pause as `make park` before it releases the servos
ROS_STATE_TIMEOUT = 0.5   # older /joint_states = no feedback for a move
ROS_STALE_AFTER = 2.0     # seconds without /joint_states before the page shows "disconnected"
ROS_SOURCE = "web"        # ArmCommand.source and the control owner name of the page
# /diagnostics from temperature_monitor: "mycobot: J<n> servo temperature", level -> temp level
DIAG_TEMP_NAME = "mycobot: J{} servo temperature"
DIAG_TEMP_LEVELS = {"ok": "normal", "warn": "warm", "error": "hot"}

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


def dh_coords(angles):
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


class _Base:
    CAPABILITIES = ("stop", "poses", "joints", "wake")

    def __init__(self):
        self._lock = threading.Lock()
        self._closed = threading.Event()
        self._state = {
            "mock": False,
            "source": "real",   # real | mock | gazebo
            "connected": False,
            "error": None,
            "angles": None,
            "coords": None,
            "temps": None,
            "temp_levels": None,
            "volts": None,      # servo supply voltage per joint, V
            "power_on": None,
            "resting": False,   # rest mode (motion refused): set by set_rest(), cleared by wake_up()
            "moving": None,
            "gripper": None,    # "open" | "closed" | None (unknown), last commanded
            "led": None,        # [r, g, b] 0-255, last commanded
            "control_owner": None,   # arm_driver's control owner ("web", "joystick", ...; "" free)
            "notice": None,     # {"t", "text"}: something the operators should know (move timed out...)
            "capabilities": list(self.CAPABILITIES),
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

    def take_control(self):
        """The page's lease was taken (or is still held). Nobody else to ask here."""

    def release_control(self):
        """The page's lease ended."""


class MockArm(_Base):
    """Synthetic arm: slow sine waves inside the joint limits, warming servos."""

    CAPABILITIES = ("stop", "poses", "joints", "wake", "rest", "gripper", "led")

    def __init__(self):
        super().__init__()
        self._t0 = time.time()
        self._temps = [32.0, 35.0, 36.0, 38.0, 34.0, 31.0]
        self._volts = [12.0] * 6
        self._stopped_until = 0.0
        self._frozen = None
        self._pose = None   # (angles, started_at, from_angles) while holding a pose
        self._update(mock=True, source="mock", connected=True, power_on=True)

    def start(self):
        threading.Thread(target=self._run, daemon=True).start()

    def _run(self):
        last_temp = 0.0
        while not self._closed.is_set():
            now = time.time()
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
                    for i, (_, hi) in enumerate(JOINT_LIMITS)
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
                # the supply sags a little while the joints move
                self._volts = [round(12.0 - (random.uniform(0.2, 0.9) if moving else random.uniform(0.0, 0.2)), 1)
                               for _ in range(6)]
                self._update(volts=self._volts)

            self._closed.wait(ANGLE_PERIOD)

    @staticmethod
    def _fake_coords(angles):
        return dh_coords(angles)

    def stop_motion(self):
        self._pose = None
        self._stopped_until = time.time() + 5

    def go_to_pose(self, name, speed=POSE_SPEED):
        if name not in POSES:
            raise ValueError("unknown pose: " + name)
        check_awake(self.snapshot()["resting"])
        self._start_move(POSES[name])

    def move_joint(self, index, angle, speed=POSE_SPEED):
        check_awake(self.snapshot()["resting"])
        target = list(self._frozen or [0.0] * 6)
        target[index] = angle
        self._start_move(target)

    def wake_up(self):
        self._update(power_on=True, resting=False)   # stays where it is

    def set_rest(self, on):
        self.stop_motion()
        self._update(resting=bool(on))

    def set_gripper(self, closed):
        check_awake(self.snapshot()["resting"])
        self._update(gripper="closed" if closed else "open")

    def set_led(self, r, g, b):
        self._update(led=[int(r), int(g), int(b)])

    def _start_move(self, target):
        self._update(power_on=True, resting=False)
        origin = list(self._frozen or [0.0] * 6)
        self._stopped_until = 0.0
        self._pose = (target, time.time(), origin)




class GazeboArm(_Base):
    """Arm simulated in Gazebo, read and commanded through ROS 2.

    Uses the shared node of ros.py, so it only works where ROS is sourced (the
    sim container). Reads sensor_msgs/JointState and commands a
    joint_trajectory_controller. There are no temperatures or servo power in
    simulation: those stay None.
    """

    def __init__(self, joint_names=None, state_topic=GZ_STATE_TOPIC, traj_topic=GZ_TRAJ_TOPIC, bridge=None):
        super().__init__()
        self.joint_names = list(joint_names or GZ_JOINT_NAMES)
        self.state_topic = state_topic
        self.traj_topic = traj_topic
        self._bridge = bridge or ros.bridge
        self._publisher = None
        self._msgs = None
        self._last_msg = 0.0
        self._last_move = 0.0
        self._update(source="gazebo", connected=False, power_on=True,
                     error="waiting for " + state_topic)

    def start(self):
        self._bridge.start()
        self._bridge.when_ready(self._setup)
        self._thread = threading.Thread(target=self._watch, daemon=True)
        self._thread.start()

    def close(self):
        super().close()
        thread = getattr(self, "_thread", None)
        if thread is not None:
            thread.join(timeout=5)

    def _setup(self, node):
        if node is None:
            self._update(connected=False,
                         error="{}, run inside the sim container".format(self._bridge.error))
            return
        from builtin_interfaces.msg import Duration
        from sensor_msgs.msg import JointState
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        node.create_subscription(JointState, self.state_topic, self._on_joint_state, 10)
        self._msgs = (Duration, JointTrajectory, JointTrajectoryPoint)
        self._publisher = node.create_publisher(JointTrajectory, self.traj_topic, 10)

    def _watch(self):
        while not self._closed.wait(0.1):
            self._check_stale()

    def _on_joint_state(self, msg):
        positions = dict(zip(msg.name, msg.position))
        missing = [n for n in self.joint_names if n not in positions]
        if missing:
            self._update(connected=False, error="joint_states has no joint " + ", ".join(missing))
            return
        angles = [round(math.degrees(positions[n]), 2) for n in self.joint_names]
        now = time.time()
        previous = self.snapshot()["angles"]
        if previous is None or max(abs(a - b) for a, b in zip(angles, previous)) > GZ_MOVING_EPS:
            self._last_move = now
        self._last_msg = now
        self._update(angles=angles, coords=dh_coords(angles), connected=True, error=None,
                     moving=now - self._last_move < 0.5)

    def _check_stale(self):
        if self._last_msg and time.time() - self._last_msg > GZ_STALE_AFTER:
            self._last_msg = 0.0
            self._update(connected=False, moving=None,
                         error="no {} for {:.0f} s, is Gazebo running?".format(
                             self.state_topic, GZ_STALE_AFTER))

    def _send(self, target, seconds):
        if self._publisher is None:
            raise RuntimeError("ROS 2 not ready")
        Duration, JointTrajectory, JointTrajectoryPoint = self._msgs
        point = JointTrajectoryPoint()
        point.positions = [math.radians(a) for a in target]
        point.time_from_start = Duration(sec=int(seconds), nanosec=int((seconds % 1) * 1e9))
        trajectory = JointTrajectory()
        trajectory.joint_names = list(self.joint_names)
        trajectory.points = [point]
        self._publisher.publish(trajectory)

    def _current(self):
        angles = self.snapshot()["angles"]
        if angles is None:
            raise RuntimeError("no joint states from Gazebo yet")
        return angles

    def _move_to(self, target, speed):
        seconds = max(GZ_MIN_MOVE_TIME,
                      max(abs(t - c) for t, c in zip(target, self._current())) / (speed * GZ_DEG_PER_SPEED))
        self._send(target, seconds)

    def stop_motion(self):
        self._send(self._current(), 0.1)   # a new trajectory replaces the running one

    def go_to_pose(self, name, speed=POSE_SPEED):
        if name not in POSES:
            raise ValueError("unknown pose: " + name)
        self._move_to([float(a) for a in POSES[name]], speed)

    def move_joint(self, index, angle, speed=POSE_SPEED):
        target = list(self._current())
        target[index] = angle
        self._move_to(target, speed)

    def wake_up(self):
        pass   # simulated servos are always on and never rest


class RosArm(_Base):
    """The real arm through arm_driver (ros2_ws/src/braco_driver/src/arm_driver.cpp).

    arm_driver owns the serial port; this source only uses its ROS interface:

        sub  /joint_states            angles (radians, joint1..joint6)
        sub  /arm/servo_temperatures  temperatures (valid = false: no answer)
        sub  /arm/read_only           rest mode (latched)
        pub  /arm/command             STOP / ALL_ANGLES, streamed (see ROS_STEP_DEG)
        pub  /arm/led                 Atom LED colour
        srv  /arm/power_on, /arm/release_servos, /arm/set_gripper, /arm/set_read_only

        sub  /arm/control_owner       who may move the arm (latched)
        sub  /diagnostics             temperature levels, from temperature_monitor
        srv  /arm/take_control        while the page's lease is held (source "web")

    A move runs in a worker thread that resends bounded steps until the arm is
    within ROS_TOLERANCE_DEG of the target, exactly like ArmMoves::moveTo(). A
    newer command or STOP cancels it, and so does losing the control (the joystick
    forced it); if the web server dies, the driver's 300 ms watchdog stops the arm.
    on_control_lost(owner) is called (from a ROS thread) when another client takes it.
    """

    CAPABILITIES = ("stop", "poses", "joints", "wake", "rest", "gripper", "led")

    def __init__(self, bridge=None):
        super().__init__()
        self._bridge = bridge or ros.bridge
        self._types = None
        self._cmd_pub = None
        self._led_pub = None
        self._clients = {}
        self._last_msg = 0.0
        self._last_move = 0.0
        self._motion = 0   # bumped by every command; a worker stops when it changes
        self._motion_lock = threading.Lock()
        self.on_control_lost = None
        self._update(source="ros", connected=False, error="waiting for arm_driver (/joint_states)")

    # ---- setup ----------------------------------------------------------------

    def start(self):
        self._bridge.start()
        self._bridge.when_ready(self._setup)
        self._thread = threading.Thread(target=self._watch, daemon=True)
        self._thread.start()

    def close(self):
        super().close()
        thread = getattr(self, "_thread", None)
        if thread is not None:
            thread.join(timeout=5)

    def _setup(self, node):
        if node is None:
            self._update(connected=False, error=self._bridge.error)
            return
        try:
            from braco_interfaces.msg import ArmCommand, ServoTemperatures, ServoVoltages
            from braco_interfaces.srv import TakeControl
        except ImportError as e:
            self._update(connected=False,
                         error="braco_interfaces not found, build and source the workspace ({})".format(e))
            return
        from diagnostic_msgs.msg import DiagnosticArray
        from rclpy.qos import DurabilityPolicy, QoSProfile
        from sensor_msgs.msg import JointState
        from std_msgs.msg import Bool, ColorRGBA, String
        from std_srvs.srv import SetBool, Trigger

        self._types = {"ArmCommand": ArmCommand, "ColorRGBA": ColorRGBA,
                       "SetBool": SetBool, "Trigger": Trigger, "TakeControl": TakeControl}
        node.create_subscription(JointState, "/joint_states", self._on_joint_state, 10)
        node.create_subscription(ServoTemperatures, "/arm/servo_temperatures", self._on_temps, 10)
        node.create_subscription(ServoVoltages, "/arm/servo_voltages", self._on_volts, 10)
        node.create_subscription(DiagnosticArray, "/diagnostics", self._on_diagnostics, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(Bool, "/arm/read_only", self._on_read_only, latched)
        node.create_subscription(String, "/arm/control_owner", self._on_control_owner, latched)
        self._cmd_pub = node.create_publisher(ArmCommand, "/arm/command", 10)
        self._led_pub = node.create_publisher(ColorRGBA, "/arm/led", 10)
        self._clients = {
            "power_on": node.create_client(Trigger, "/arm/power_on"),
            "release_servos": node.create_client(Trigger, "/arm/release_servos"),
            "set_gripper": node.create_client(SetBool, "/arm/set_gripper"),
            "set_read_only": node.create_client(SetBool, "/arm/set_read_only"),
            "take_control": node.create_client(TakeControl, "/arm/take_control"),
        }

    # ---- telemetry --------------------------------------------------------------

    def _on_joint_state(self, msg):
        if len(msg.position) < 6:
            return
        names = list(msg.name)
        wanted = ["joint{}".format(i) for i in range(1, 7)]
        if all(n in names for n in wanted):
            positions = [msg.position[names.index(n)] for n in wanted]
        else:
            positions = list(msg.position[:6])
        angles = [round(math.degrees(p), 2) for p in positions]
        now = time.time()
        previous = self.snapshot()["angles"]
        if previous is None or max(abs(a - b) for a, b in zip(angles, previous)) > GZ_MOVING_EPS:
            self._last_move = now
        self._last_msg = now
        self._update(angles=angles, coords=dh_coords(angles), connected=True, error=None,
                     moving=now - self._last_move < 0.5)

    def _on_temps(self, msg):
        # The levels come from /diagnostics (temperature_monitor), not from a copy of the limits
        self._update(temps=[int(c) for c in msg.celsius] if msg.valid else None)

    def _on_volts(self, msg):
        self._update(volts=[round(float(v), 1) for v in msg.volts] if msg.valid else None)

    def _on_diagnostics(self, msg):
        levels = list(self.snapshot()["temp_levels"] or [None] * 6)
        seen = False
        for status in msg.status:
            if status.name == "mycobot: servo temperatures":   # STALE: no data at all
                self._update(temp_levels=None)
                return
            for j in range(6):
                if status.name == DIAG_TEMP_NAME.format(j + 1):
                    levels[j] = DIAG_TEMP_LEVELS.get(level_name(status.level))
                    seen = True
        if seen:
            self._update(temp_levels=levels)

    def _on_read_only(self, msg):
        self._update(resting=bool(msg.data))

    def _on_control_owner(self, msg):
        owner = str(msg.data)
        self._update(control_owner=owner)
        if owner != ROS_SOURCE:
            self._cancel()   # a running move stops streaming (no STOP: it is not ours to send)
            if owner and self.on_control_lost is not None:
                self.on_control_lost(owner)

    def _watch(self):
        while not self._closed.wait(0.2):
            if self._last_msg and time.time() - self._last_msg > ROS_STALE_AFTER:
                self._last_msg = 0.0
                self._update(connected=False, moving=None,
                             error="no /joint_states for {:.0f} s, is arm_driver running? (make logs)".format(
                                 ROS_STALE_AFTER))

    def _fresh_angles(self):
        if not self._last_msg or time.time() - self._last_msg > ROS_STATE_TIMEOUT:
            return None
        return self.snapshot()["angles"]

    def _notice(self, text):
        self._update(notice={"t": time.time(), "text": text})

    # ---- low level ----------------------------------------------------------------

    def _ready(self):
        if self._cmd_pub is None:
            raise RuntimeError(self._bridge.error or "ROS 2 not ready")

    def _service(self, name, data=None):
        self._ready()
        srv = self._types["SetBool" if data is not None else "Trigger"]
        request = srv.Request(data=data) if data is not None else srv.Request()
        result = self._bridge.call(self._clients[name], request)
        if not result.success:
            raise RuntimeError(result.message or name + " failed")
        return result.message

    def _publish_angles(self, angles, speed):
        ArmCommand = self._types["ArmCommand"]
        self._cmd_pub.publish(ArmCommand(type=ArmCommand.ALL_ANGLES, angles_deg=[float(a) for a in angles],
                                         speed=int(speed), source=ROS_SOURCE))

    def _publish_stop(self):
        ArmCommand = self._types["ArmCommand"]
        self._cmd_pub.publish(ArmCommand(type=ArmCommand.STOP, source=ROS_SOURCE))

    def _has_control(self):
        return self.snapshot()["control_owner"] == ROS_SOURCE

    def _cancel(self):
        with self._motion_lock:
            self._motion += 1
            return self._motion

    def _start_move(self, target, speed, on_arrival=None):
        """Stream `target` in a worker thread; on_arrival() runs there once it is reached."""
        token = self._cancel()
        threading.Thread(target=self._stream, args=(token, list(target), speed, on_arrival),
                         daemon=True).start()
        return token

    def _park_finish(self, token):
        """Like `make park`: let the arm settle, release the servos, then enter rest mode."""
        if self._closed.wait(ROS_PARK_SETTLE_S) or token != self._motion or not self._has_control():
            return   # STOP, a newer command or a lost control: do not release a moving arm
        try:
            self._service("release_servos")
            self._update(power_on=False)
            self._service("set_read_only", True)
            self._update(resting=True)
        except Exception as e:
            self._notice("park: the arm arrived but the servos were not released ({})".format(e))

    def _stream(self, token, target, speed, on_arrival=None):
        end = time.monotonic() + ROS_MOVE_TIMEOUT
        while token == self._motion and not self._closed.is_set():
            angles = self._fresh_angles()
            if angles is not None:
                deviation = max(abs(a - p) for a, p in zip(angles, target))
                if deviation <= ROS_TOLERANCE_DEG:
                    if on_arrival is not None:
                        on_arrival(token)
                    return
                # Same fraction for every joint, so they all arrive together
                scale = min(1.0, ROS_STEP_DEG / deviation)
                self._publish_angles([round(a + (p - a) * scale, 1) for a, p in zip(angles, target)],
                                     speed)
            if time.monotonic() >= end:
                break
            self._closed.wait(ROS_RESEND_S)
        if token == self._motion and not self._closed.is_set() and self._has_control():
            self._publish_stop()
            self._notice("target not reached in {:.0f} s (or no feedback): arm stopped".format(ROS_MOVE_TIMEOUT))

    # ---- control owner ----------------------------------------------------------------

    def _control_call(self, release=False):
        self._ready()
        TakeControl = self._types["TakeControl"]
        request = TakeControl.Request(source=ROS_SOURCE, force=False, release=release)
        return self._bridge.call(self._clients["take_control"], request)

    def take_control(self):
        """Become arm_driver's control owner, or renew it. ControlError if another client has it."""
        result = self._control_call()
        self._update(control_owner=result.owner)
        if not result.success:
            raise ControlError(result.message or "the arm is controlled by " + result.owner)

    def release_control(self):
        """Give the control back; best effort (arm_driver frees it after control_idle_s anyway)."""
        self._cancel()
        try:
            result = self._control_call(release=True)
            self._update(control_owner=result.owner)
        except Exception as e:   # driver gone or not ready: nothing to give back
            print("braco: release control: {}".format(e))

    # ---- commands -------------------------------------------------------------------

    def _check_can_move(self):
        self._ready()
        if self._fresh_angles() is None:
            raise RuntimeError("no fresh /joint_states from arm_driver")
        check_awake(self.snapshot()["resting"])

    def stop_motion(self):
        self._ready()
        self._cancel()
        self._publish_stop()

    def go_to_pose(self, name, speed=POSE_SPEED):
        if name not in POSES:
            raise ValueError("unknown pose: " + name)
        self._check_can_move()
        self._start_move(POSES[name], speed, self._park_finish if name == "park" else None)

    def move_joint(self, index, angle, speed=POSE_SPEED):
        self._check_can_move()
        target = list(self._fresh_angles())
        target[index] = float(angle)
        self._start_move(target, speed)

    def wake_up(self):
        """Leave rest mode and power the servos on. The arm stays where it is."""
        self._ready()
        self._cancel()
        self._service("set_read_only", False)
        self._service("power_on")
        self._update(power_on=True, resting=False)

    def set_rest(self, on):
        """Rest mode on/off without moving: motion is refused, temperatures keep coming."""
        self._ready()
        if on:
            self._cancel()
        self._service("set_read_only", bool(on))
        self._update(resting=bool(on))

    def set_gripper(self, closed):
        self._ready()
        check_awake(self.snapshot()["resting"])
        self._service("set_gripper", bool(closed))
        self._update(gripper="closed" if closed else "open")

    def set_led(self, r, g, b):
        self._ready()
        ColorRGBA = self._types["ColorRGBA"]
        self._led_pub.publish(ColorRGBA(r=r / 255.0, g=g / 255.0, b=b / 255.0, a=1.0))
        self._update(led=[int(r), int(g), int(b)])
