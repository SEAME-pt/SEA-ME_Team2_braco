#!/usr/bin/env python3
"""
Joystick control of all 6 joints of the myCobot 280 (Jetson Nano).

Safety model: step-ahead (bounded-lead) jogging
------------------------------------------------
The arm is never given a far-away goal. On every loop tick the script reads
the real joint angles and commands a target only a few degrees ahead of them.
If commands stop arriving (crash, kill -9, frozen process, lost serial data,
joystick unplugged), each joint coasts at most the active mode's lead_max
(jogging) or HOME_LEAD_DEG (homing) and then stops on its own.

Behaviour:
  * Stick speed is proportional to deflection (continuous, with an expo curve
    for fine control near the centre).
  * Five speed modes, cycled with button 9. The Atom LED shows the active
    mode: blue = slow (12%), green = medium (30%), purple = 50%,
    white = 75%, red = 100%.
  * Near a joint limit, speed and step shrink with the remaining distance.
  * Idle joints are latched and held while other joints jog (prevents J4 sag
    while J6 rotates the off-centre gripper).
  * Homing is one click: press X (button 3) once and all joints walk to 0.
    Press X again, or touch any stick / bumper, to abort.

Mapping:
    Left stick  X / Y  joint 1 / joint 2 (base yaw [inverted] / shoulder pitch)
    Right stick Y / X  joint 4 / joint 5
    L1 / R1 (6 / 7)    joint 6 (tool rotation), exclusive while held
    D-pad Y            joint 3 (elbow pitch)
    A / B              close / open gripper
    X (click)          home all joints to 0 (click again to abort)
    Button 9           cycle speed: blue -> green -> purple -> white -> red -> blue ...
    Ctrl+C             quit (safely stops the arm)

Usage:
    python3 joystick_all_joints.py           # control the arm
    python3 joystick_all_joints.py --debug   # print events, targets, hold corrections
"""

import os
import signal
import struct
import sys
import time

from pymycobot import MyCobot280

SERIAL_PORT = '/dev/ttyTHS1'
BAUD_RATE = 1000000
JOYSTICK_DEVICE = '/dev/input/js0'

# ---- Speed modes (pymycobot speed units 1..100 = percent) ------------------
# max_speed:    jog speed at full stick deflection
# lead_max:     step-ahead at full deflection (= worst-case coast if commands stop)
# bumper_speed: fixed J6 speed for L1/R1 (lower = less reaction torque on J4/J5)
# color:        Atom LED colour (R, G, B)
SPEED_MODES = [
    {'name': 'slow', 'max_speed': 12, 'lead_max': 5.0, 'bumper_speed': 10, 'color': (0, 0, 255)},
    {
        'name': 'medium',
        'max_speed': 30,
        'lead_max': 10.0,
        'bumper_speed': 20,
        'color': (0, 255, 0),
    },
    {
        'name': '50%',
        'max_speed': 50,
        'lead_max': 15.0,
        'bumper_speed': 30,
        'color': (128, 0, 255),
    },  # purple
    {
        'name': '75%',
        'max_speed': 75,
        'lead_max': 20.0,
        'bumper_speed': 40,
        'color': (255, 255, 255),
    },  # white
    {
        'name': '100%',
        'max_speed': 100,
        'lead_max': 25.0,
        'bumper_speed': 50,
        'color': (255, 0, 0),
    },  # red
]
START_MODE = 1  # index into SPEED_MODES (1 = medium, original behaviour)

MIN_SPEED = 3  # speed just outside the deadzone (fine positioning)
STICK_EXPO = 1.8  # 1.0 = linear; >1 = more resolution near centre
GRIPPER_SPEED = 50  # gripper is a separate actuator

# ---- Step-ahead lead --------------------------------------------------------
LEAD_MIN_DEG = 1.0  # lead at MIN_SPEED (raise if small moves pulse)

# ---- Soft limit zone --------------------------------------------------------
SLOW_ZONE_DEG = 25.0  # start slowing this far from a limit
MIN_LIMIT_FACTOR = 0.15  # slowest scaling inside the zone (keeps a creep)
AT_LIMIT_DEG = 0.3  # closer than this -> treat as at the limit

# ---- One-click homing -------------------------------------------------------
HOME_SPEED = 60  # "fast home", independent of the jog speed mode
HOME_LEAD_DEG = 15.0  # bounded step per tick while homing
HOME_TOLERANCE_DEG = 1.0  # all joints within this of 0 -> home reached
HOME_MAX_BAD_READS = 3  # consecutive invalid feedback reads before aborting

# ---- Idle-joint hold --------------------------------------------------------
HOLD_TOLERANCE_DEG = 1.5  # re-command an idle joint if it drifts more than this
HOLD_RESEND_PERIOD = 0.3  # seconds between hold corrections per joint
HOLD_SPEED = 10  # gentle correction speed

# ---- Input ------------------------------------------------------------------
AXIS_DEADZONE = 5000  # axis range is -32767..32767
AXIS_MAX = 32767.0
BUMPERS_EXCLUSIVE = True  # while L1/R1 held, ignore all stick axes

COMMAND_GAP = 0.05  # delay after each serial command
LOOP_PERIOD = 0.02  # idle sleep per loop iteration

# axis number -> (joint 1..6, direction: 1 or -1)
AXIS_TO_JOINT = {
    0: (1, -1),  # left stick X  -> joint 1 (base yaw) [inverted]
    1: (2, 1),  # left stick Y  -> joint 2 (shoulder pitch)
    3: (4, -1),  # right stick vertical   -> joint 4
    2: (5, -1),  # right stick horizontal -> joint 5
    7: (3, 1),  # d-pad Y       -> joint 3 (elbow)
}

# Analog trigger axes. Button 9 (R2) also drives axis 4 (-32767 released,
# +32767 pressed), so these axes are ignored completely and never move a joint.
TRIGGER_AXES = {4, 5}

BUTTON_GRIPPER_CLOSE = 0  # A
BUTTON_GRIPPER_OPEN = 1  # B
BUTTON_HOME = 3  # X (click)
BUTTON_L1 = 6  # J6 negative
BUTTON_R1 = 7  # J6 positive
BUTTON_SPEED_MODE = 9  # cycle speed mode (never moves the arm)

# Some controllers send axis events together with button 9 (e.g. an analog
# trigger that reports both an axis and a button). Any axis that changes while
# button 9 is held, or within this window around its press/release, is ignored
# until that axis returns to its centre (deadzone).
SPEED_BUTTON_GUARD_MS = 250

# Safe mechanical limits (deg) to avoid firmware command rejections
LIMITS = [None, 160, 130, 140, 120, 135, 170]

EVENT_FORMAT = 'IhBB'  # time (ms), value, type, number
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)
TYPE_BUTTON = 0x01
TYPE_AXIS = 0x02
TYPE_INIT = 0x80


def read_events(fd):
    """Return all pending events in the joystick buffer."""
    events = []
    while True:
        try:
            data = os.read(fd, EVENT_SIZE)
        except BlockingIOError:
            break
        if len(data) < EVENT_SIZE:
            break
        t_ms, value, ev_type, number = struct.unpack(EVENT_FORMAT, data)
        if ev_type & TYPE_INIT:
            continue
        events.append((t_ms, ev_type, number, value))
    return events


def stick_ratio(raw_value):
    """Deflection beyond the deadzone, 0..1, shaped by the expo curve."""
    magnitude = abs(raw_value)
    if magnitude <= AXIS_DEADZONE:
        return 0.0
    ratio = min(1.0, (magnitude - AXIS_DEADZONE) / (AXIS_MAX - AXIS_DEADZONE))
    return ratio**STICK_EXPO


def motion_for_ratio(ratio, mode):
    """Map a 0..1 command ratio to (speed, lead) for the given speed mode."""
    speed = MIN_SPEED + ratio * (mode['max_speed'] - MIN_SPEED)
    lead = LEAD_MIN_DEG + ratio * (mode['lead_max'] - LEAD_MIN_DEG)
    return speed, lead


def bumper_ratio(mode):
    """Ratio that yields the mode's fixed J6 bumper speed."""
    span = float(mode['max_speed'] - MIN_SPEED)
    return max(0.0, min(1.0, (mode['bumper_speed'] - MIN_SPEED) / span))


def shape_near_limit(joint, angle, direction, speed, lead):
    """Scale speed and lead by the distance left to the limit we move toward.

    Returns (target, speed) or None if the joint is already at the limit.
    Moving away from a limit is never slowed.
    """
    lim = LIMITS[joint]
    remaining = (lim - angle) if direction > 0 else (angle + lim)
    if remaining <= AT_LIMIT_DEG:
        return None
    if remaining >= SLOW_ZONE_DEG:
        factor = 1.0
    else:
        factor = max(MIN_LIMIT_FACTOR, remaining / SLOW_ZONE_DEG)
    speed = max(1, int(round(speed * factor)))
    lead = min(lead * factor, remaining)  # never step past the limit
    return angle + direction * lead, speed


def read_angles(mc):
    """Return the 6 current joint angles, or None if the reading is invalid."""
    try:
        angles = mc.get_angles()
    except Exception:
        return None
    if (
        isinstance(angles, list)
        and len(angles) == 6
        and all(isinstance(a, (int, float)) for a in angles)
    ):
        return angles
    return None


def set_led(mc, mode):
    """Show the speed mode on the Atom LED."""
    r, g, b = mode['color']
    try:
        mc.set_color(r, g, b)
    except Exception as error:
        print('!!! could not set LED colour (%s)' % error)
    time.sleep(COMMAND_GAP)


def wanted_motion(axis_state, button_state, mode, blocked_axes=()):
    """Return desired {joint: (direction, speed, lead)}.

    Axes in blocked_axes (side effects of button 9) never produce motion.
    """
    wanted = {}

    l1 = button_state.get(BUTTON_L1, 0) == 1
    r1 = button_state.get(BUTTON_R1, 0) == 1
    bumper_dir = -1 if (l1 and not r1) else (1 if (r1 and not l1) else 0)
    if bumper_dir:
        speed, lead = motion_for_ratio(bumper_ratio(mode), mode)
        wanted[6] = (bumper_dir, speed, lead)
        if BUMPERS_EXCLUSIVE:
            return wanted  # ignore any (ghost) stick input while J6 rotates

    for axis, (joint, direction) in AXIS_TO_JOINT.items():
        if axis in blocked_axes:
            continue
        value = axis_state.get(axis, 0)
        ratio = stick_ratio(value)
        if ratio > 0.0:
            sign = direction if value > 0 else -direction
            speed, lead = motion_for_ratio(ratio, mode)
            wanted[joint] = (sign, speed, lead)

    return wanted


def quit_on_signal(signum, frame):
    raise KeyboardInterrupt


def main():
    debug = '--debug' in sys.argv

    signal.signal(signal.SIGTERM, quit_on_signal)
    signal.signal(signal.SIGHUP, quit_on_signal)

    print('1/2 Opening joystick:', JOYSTICK_DEVICE)
    fd = os.open(JOYSTICK_DEVICE, os.O_RDONLY | os.O_NONBLOCK)
    print('    OK')

    print('2/2 Connecting to arm on:', SERIAL_PORT)
    mc = MyCobot280(SERIAL_PORT, BAUD_RATE)
    time.sleep(1)
    mc.power_on()
    time.sleep(0.5)
    print('    Current angles:', mc.get_angles())

    mode_index = START_MODE
    mode = SPEED_MODES[mode_index]
    set_led(mc, mode)

    print(
        'Ready. Speed mode: %s (cap %d %%, max coast %.0f deg). '
        'Button 9 changes speed, X homes, Ctrl+C quits.'
        % (mode['name'], mode['max_speed'], mode['lead_max'])
    )

    axis_state = {}
    button_state = {}
    active = set()  # joints commanded on the previous tick
    hold = {}  # idle joint -> latched angle during a jog session
    last_hold_send = {}  # joint -> time of last hold command
    limit_reported = set()  # joints whose "at limit" message was printed
    homing = False
    bad_reads = 0
    axis_changed_ms = {}  # axis -> joystick time (ms) of its last event
    blocked_axes = set()  # axes ignored because they fired with button 9
    speed_btn_ms = None  # joystick time of the last button 9 press/release

    def stop_arm():
        mc.stop()
        time.sleep(COMMAND_GAP)
        active.clear()
        hold.clear()

    try:
        while True:
            # 1. Drain joystick events
            home_clicked = False
            for t_ms, ev_type, number, value in read_events(fd):
                if ev_type == TYPE_AXIS:
                    if number in TRIGGER_AXES:
                        continue  # trigger analog value, never a joint command
                    axis_state[number] = value
                    axis_changed_ms[number] = t_ms
                    near_speed_btn = button_state.get(BUTTON_SPEED_MODE, 0) == 1 or (
                        speed_btn_ms is not None
                        and abs(t_ms - speed_btn_ms) <= SPEED_BUTTON_GUARD_MS
                    )
                    if near_speed_btn and abs(value) > AXIS_DEADZONE:
                        if number not in blocked_axes and debug:
                            print('axis   %d blocked (fired with button 9)' % number)
                        blocked_axes.add(number)
                    elif number in blocked_axes and abs(value) <= AXIS_DEADZONE:
                        blocked_axes.discard(number)  # back at centre -> usable
                        if debug:
                            print('axis   %d unblocked' % number)
                    if debug:
                        print('axis   %d = %d' % (number, value))
                elif ev_type == TYPE_BUTTON:
                    button_state[number] = value
                    if debug:
                        print('button %d = %d' % (number, value))
                    if number == BUTTON_SPEED_MODE:
                        speed_btn_ms = t_ms
                        # Block axes that moved just before button 9 fired
                        # (analog triggers move their axis first).
                        for ax, ax_ms in axis_changed_ms.items():
                            if (
                                t_ms - ax_ms <= SPEED_BUTTON_GUARD_MS
                                and abs(axis_state.get(ax, 0)) > AXIS_DEADZONE
                            ):
                                blocked_axes.add(ax)
                    if value == 1:
                        if number == BUTTON_GRIPPER_CLOSE:
                            print('>>> close gripper')
                            mc.set_gripper_state(1, GRIPPER_SPEED)
                        elif number == BUTTON_GRIPPER_OPEN:
                            print('>>> open gripper')
                            mc.set_gripper_state(0, GRIPPER_SPEED)
                        elif number == BUTTON_HOME:
                            home_clicked = True
                        elif number == BUTTON_SPEED_MODE:
                            mode_index = (mode_index + 1) % len(SPEED_MODES)
                            mode = SPEED_MODES[mode_index]
                            set_led(mc, mode)
                            print(
                                '>>> speed mode: %s (cap %d %%, max coast %.0f deg)'
                                % (mode['name'], mode['max_speed'], mode['lead_max'])
                            )

            wanted = wanted_motion(axis_state, button_state, mode, blocked_axes)

            # 2. Homing state transitions (click toggles, any jog input aborts)
            if home_clicked:
                if homing:
                    print('>>> homing cancelled')
                    homing = False
                    stop_arm()
                else:
                    print('>>> homing (click X again or touch a stick to abort)')
                    homing = True
                    bad_reads = 0
                    active.clear()
                    hold.clear()
            if homing and wanted:
                print('>>> homing aborted by stick/bumper input')
                homing = False
                stop_arm()

            # 3. Homing: bounded, synchronised steps toward all-zero
            if homing:
                angles = read_angles(mc)
                if angles is None:
                    bad_reads += 1
                    mc.stop()
                    time.sleep(COMMAND_GAP)
                    if bad_reads >= HOME_MAX_BAD_READS:
                        print('!!! no valid angle feedback, homing aborted')
                        homing = False
                    time.sleep(LOOP_PERIOD)
                    continue
                bad_reads = 0
                max_dev = max(abs(a) for a in angles)
                if max_dev <= HOME_TOLERANCE_DEG:
                    print('>>> home reached')
                    homing = False
                else:
                    # Same fraction for every joint -> they all arrive together
                    scale = min(1.0, HOME_LEAD_DEG / max_dev)
                    targets = [round(a * (1.0 - scale), 1) for a in angles]
                    if debug:
                        print('home targets', targets)
                    mc.send_angles(targets, HOME_SPEED)
                    time.sleep(COMMAND_GAP)
                time.sleep(LOOP_PERIOD)
                continue

            # 4. Nothing requested -> stop once, then idle (no serial traffic)
            if not wanted:
                if active:
                    print('>>> stop')
                    stop_arm()
                hold.clear()
                limit_reported.clear()
                time.sleep(LOOP_PERIOD)
                continue

            # 5. Jogging requires fresh feedback
            angles = read_angles(mc)
            if angles is None:
                print('!!! no valid angle feedback, stopping')
                stop_arm()
                time.sleep(LOOP_PERIOD)
                continue

            now = time.monotonic()
            moving = set(wanted)
            released = active - moving
            limit_reported &= moving

            # 6a. Latch idle joints (at session start or on release)
            for j in range(1, 7):
                if j in moving:
                    hold.pop(j, None)
                elif j not in hold:
                    hold[j] = angles[j - 1]

            # 6b. Released joints: pin at the latched angle
            for j in sorted(released):
                mc.send_angle(j, round(hold[j], 1), HOLD_SPEED)
                last_hold_send[j] = now
                time.sleep(COMMAND_GAP)

            # 6c. Idle joints that drifted (e.g. J4 sag while J6 rotates)
            for j in sorted(hold):
                if j in released:
                    continue
                drift = angles[j - 1] - hold[j]
                if (
                    abs(drift) > HOLD_TOLERANCE_DEG
                    and now - last_hold_send.get(j, 0.0) >= HOLD_RESEND_PERIOD
                ):
                    if debug:
                        print('hold J%d: drift %+.1f, back to %.1f' % (j, drift, hold[j]))
                    mc.send_angle(j, round(hold[j], 1), HOLD_SPEED)
                    last_hold_send[j] = now
                    time.sleep(COMMAND_GAP)

            # 6d. Moving joints: proportional speed, slowed near limits
            for joint, (direction, speed, lead) in sorted(wanted.items()):
                shaped = shape_near_limit(joint, angles[joint - 1], direction, speed, lead)
                if shaped is None:
                    if joint not in limit_reported:
                        print('>>> J%d at limit' % joint)
                        limit_reported.add(joint)
                    continue
                limit_reported.discard(joint)
                target, spd = shaped
                if debug:
                    print('J%d %.1f -> %.1f @%d' % (joint, angles[joint - 1], target, spd))
                mc.send_angle(joint, round(target, 1), spd)
                time.sleep(COMMAND_GAP)

            if moving != active:
                print('>>> jogging joints', sorted(moving))
            active = moving
            time.sleep(LOOP_PERIOD)

    except KeyboardInterrupt:
        print('\nQuitting.')
    except OSError as error:
        print('\nJoystick disconnected (%s).' % error)
    finally:
        mc.stop()
        time.sleep(0.2)
        print('Arm stopped. Final angles:', mc.get_angles())
        os.close(fd)


if __name__ == '__main__':
    main()
