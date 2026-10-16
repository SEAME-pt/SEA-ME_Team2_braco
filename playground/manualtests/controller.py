#!/usr/bin/env python3
"""
Joystick control of all 6 joints of the myCobot 280 (Jetson Nano).

Safety model: step-ahead (bounded-lead) jogging
------------------------------------------------
The arm is never given a far-away goal. On every loop tick the script reads
the real joint angles and commands a target only a few degrees ahead of them,
in the direction you are pushing. The commands have to keep arriving for the
joint to keep moving. If they stop for any reason (crash, kill -9, frozen
process, lost serial data, joystick unplugged), each joint coasts at most
LEAD_MAX_DEG degrees and then stops on its own.

Homing works the same way: Y is hold-to-run. Holding Y walks every joint
toward 0 in bounded steps; releasing Y stops the arm where it is.

All arm motion is capped at 30 % speed (SPEED_CAP).

Mapping:
    Left stick  X / Y  joint 1 / joint 2 (base yaw [inverted] / shoulder pitch)
    Right stick Y / X  joint 4 / joint 5 (vertical = joint 4 / horizontal = joint 5)
    L1 / R1 (6 / 7)    joint 6 (tool/gripper rotation)
    D-pad Y            joint 3 (elbow pitch)
    A / B              close / open gripper
    Y (hold)           all joints toward 0 (release to stop)
    Ctrl+C             quit (safely stops the arm)

Usage:
    python3 joystick_all_joints.py           # control the arm
    python3 joystick_all_joints.py --debug   # print joystick events and targets
"""

import os
import signal
import struct
import sys
import time

from pymycobot import MyCobot280

SERIAL_PORT = "/dev/ttyTHS1"
BAUD_RATE = 1000000
JOYSTICK_DEVICE = "/dev/input/js0"

# Speed parameters (pymycobot speed units are 1..100, i.e. percent)
SPEED_CAP = 30            # global cap: no arm motion faster than 30 %
MIN_SPEED = 10            # minimum speed upon exiting deadzone
MAX_SPEED = SPEED_CAP     # maximum speed at full stick deflection
BUMPER_SPEED = SPEED_CAP  # fixed speed for L1 / R1 shoulder buttons
HOME_SPEED = SPEED_CAP
GRIPPER_SPEED = 50        # gripper is a separate actuator; not affected by SPEED_CAP
SPEED_STEP = 5            # quantization step for stick speeds

# Step-ahead parameters.
# LEAD_MAX_DEG is the worst-case distance a joint can travel after commands stop.
# If motion pulses (start/stop) while a stick is held, the arm is reaching the
# target before the next command arrives: raise LEAD_MAX_DEG a little.
LEAD_MIN_DEG = 3.0        # lead at MIN_SPEED
LEAD_MAX_DEG = 10.0       # lead at SPEED_CAP
HOME_TOLERANCE_DEG = 1.0  # all joints within this of 0 -> home reached

AXIS_THRESHOLD = 16000    # deadzone (axis range is -32767 to 32767)
COMMAND_GAP = 0.05        # safety delay between serial commands
LOOP_PERIOD = 0.02        # idle sleep per loop iteration

# axis number -> (joint 1..6, direction: 1 or -1)
AXIS_TO_JOINT = {
    0: (1, -1),  # left stick X  -> joint 1 (base yaw) [inverted]
    1: (2, 1),   # left stick Y  -> joint 2 (shoulder pitch)
    3: (4, -1),  # right stick vertical   -> joint 4 (up / down)
    4: (5, -1),  # right stick horizontal -> joint 5 (pre-gripper wrist: left / right)
    7: (3, 1),   # d-pad Y       -> joint 3 (elbow)
}

# Controller buttons
BUTTON_GRIPPER_CLOSE = 0  # A
BUTTON_GRIPPER_OPEN = 1   # B
BUTTON_HOME = 3           # Y (hold-to-run)
BUTTON_L1 = 6             # L1 -> Joint 6 counter-clockwise / negative
BUTTON_R1 = 7             # R1 -> Joint 6 clockwise / positive

# Safe mechanical limits to avoid firmware command rejections
LIMITS = [None, 160, 130, 140, 120, 135, 170]

EVENT_FORMAT = "IhBB"  # time (ms), value, type, number
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
        _, value, ev_type, number = struct.unpack(EVENT_FORMAT, data)
        if ev_type & TYPE_INIT:
            continue
        events.append((ev_type, number, value))
    return events


def calculate_speed(raw_value):
    """Calculate proportional speed based on stick deflection beyond deadzone."""
    magnitude = abs(raw_value)
    if magnitude <= AXIS_THRESHOLD:
        return 0
    # Normalized ratio between 0.0 and 1.0 above deadzone
    ratio = (magnitude - AXIS_THRESHOLD) / (32767.0 - AXIS_THRESHOLD)
    raw_speed = MIN_SPEED + ratio * (MAX_SPEED - MIN_SPEED)
    quantized_speed = int(round(raw_speed / SPEED_STEP) * SPEED_STEP)
    return max(MIN_SPEED, min(SPEED_CAP, quantized_speed))


def lead_for_speed(spd):
    """Degrees ahead of the current angle to command; faster speed -> longer lead."""
    ratio = (spd - MIN_SPEED) / float(SPEED_CAP - MIN_SPEED)
    ratio = max(0.0, min(1.0, ratio))
    return LEAD_MIN_DEG + ratio * (LEAD_MAX_DEG - LEAD_MIN_DEG)


def clamp_to_limit(joint, angle):
    """Keep a target inside the joint's safe range."""
    lim = LIMITS[joint]
    return max(-lim, min(lim, angle))


def step_toward(current, goal, max_step):
    """Move from current toward goal by at most max_step degrees."""
    delta = goal - current
    if abs(delta) <= max_step:
        return goal
    return current + max_step if delta > 0 else current - max_step


def read_angles(mc):
    """Return the 6 current joint angles, or None if the reading is invalid.

    pymycobot can return -1, [] or a short list when the serial link hiccups.
    Without valid feedback we refuse to compute new targets.
    """
    try:
        angles = mc.get_angles()
    except Exception:
        return None
    if (isinstance(angles, list) and len(angles) == 6
            and all(isinstance(a, (int, float)) for a in angles)):
        return angles
    return None


def wanted_motion(axis_state, button_state):
    """Return desired {joint: (direction, speed)} for axes and L1/R1 buttons."""
    wanted = {}

    for axis, (joint, direction) in AXIS_TO_JOINT.items():
        value = axis_state.get(axis, 0)
        spd = calculate_speed(value)
        if spd > 0:
            dir_factor = direction if value > 0 else -direction
            wanted[joint] = (dir_factor, spd)

    l1_active = button_state.get(BUTTON_L1, 0) == 1
    r1_active = button_state.get(BUTTON_R1, 0) == 1

    if l1_active and not r1_active:
        wanted[6] = (-1, BUMPER_SPEED)
    elif r1_active and not l1_active:
        wanted[6] = (1, BUMPER_SPEED)

    return wanted


def quit_on_signal(signum, frame):
    raise KeyboardInterrupt


def main():
    debug = "--debug" in sys.argv

    signal.signal(signal.SIGTERM, quit_on_signal)
    signal.signal(signal.SIGHUP, quit_on_signal)

    print("1/2 Opening joystick:", JOYSTICK_DEVICE)
    fd = os.open(JOYSTICK_DEVICE, os.O_RDONLY | os.O_NONBLOCK)
    print("    OK")

    print("2/2 Connecting to arm on:", SERIAL_PORT)
    mc = MyCobot280(SERIAL_PORT, BAUD_RATE)
    time.sleep(1)
    mc.power_on()
    time.sleep(0.5)
    print("    Current angles:", mc.get_angles())

    print("Ready. Step-ahead jogging, speed cap %d %%, max coast %.0f deg. "
          "Hold Y to home. Ctrl+C quits." % (SPEED_CAP, LEAD_MAX_DEG))
    axis_state = {}
    button_state = {}
    active = set()          # joints commanded on the previous tick
    homing = False          # True while Y is held and homing is in progress
    home_reported = False

    try:
        while True:
            # 1. Drain joystick events
            for ev_type, number, value in read_events(fd):
                if ev_type == TYPE_AXIS:
                    axis_state[number] = value
                    if debug:
                        print("axis   %d = %d" % (number, value))
                elif ev_type == TYPE_BUTTON:
                    button_state[number] = value
                    if debug:
                        print("button %d = %d" % (number, value))
                    if value == 1:
                        if number == BUTTON_GRIPPER_CLOSE:
                            print(">>> close gripper")
                            mc.set_gripper_state(1, GRIPPER_SPEED)
                        elif number == BUTTON_GRIPPER_OPEN:
                            print(">>> open gripper")
                            mc.set_gripper_state(0, GRIPPER_SPEED)

            # 2. Decide what is requested (homing has priority over the sticks)
            home_held = button_state.get(BUTTON_HOME, 0) == 1
            wanted = {} if home_held else wanted_motion(axis_state, button_state)

            # 3. Y released during homing -> stop where we are
            if homing and not home_held:
                print(">>> home released, stop")
                mc.stop()
                time.sleep(COMMAND_GAP)
                active.clear()
                homing = False

            # 4. Nothing requested -> stop once, then idle (no serial traffic)
            if not home_held and not wanted:
                if active:
                    print(">>> stop")
                    mc.stop()
                    time.sleep(COMMAND_GAP)
                    active.clear()
                time.sleep(LOOP_PERIOD)
                continue

            # 5. Motion requested -> fresh feedback is mandatory
            angles = read_angles(mc)
            if angles is None:
                print("!!! no valid angle feedback, stopping")
                mc.stop()
                time.sleep(COMMAND_GAP)
                active.clear()
                time.sleep(LOOP_PERIOD)
                continue

            # 6a. Hold-to-run homing: every joint steps toward 0
            if home_held:
                if not homing:
                    print(">>> homing (keep holding Y)")
                    homing = True
                    home_reported = False
                    active.clear()
                if all(abs(a) <= HOME_TOLERANCE_DEG for a in angles):
                    if not home_reported:
                        print(">>> home reached")
                        home_reported = True
                else:
                    targets = [round(step_toward(a, 0.0, LEAD_MAX_DEG), 1)
                               for a in angles]
                    if debug:
                        print("home targets", targets)
                    mc.send_angles(targets, HOME_SPEED)
                    time.sleep(COMMAND_GAP)
                time.sleep(LOOP_PERIOD)
                continue

            # 6b. Jogging: released joints hold their current angle
            #     (per-joint stop, so the other joints keep moving smoothly)
            for joint in sorted(active - set(wanted)):
                mc.send_angle(joint, round(angles[joint - 1], 1), SPEED_CAP)
                time.sleep(COMMAND_GAP)

            # 6c. Jogging: held joints get a target a short lead ahead
            for joint, (direction, spd) in wanted.items():
                lead = lead_for_speed(spd)
                target = clamp_to_limit(joint, angles[joint - 1] + direction * lead)
                if debug:
                    print("J%d %.1f -> %.1f @%d" % (joint, angles[joint - 1], target, spd))
                mc.send_angle(joint, round(target, 1), spd)
                time.sleep(COMMAND_GAP)

            if set(wanted) != active:
                print(">>> jogging joints", sorted(wanted))
            active = set(wanted)
            time.sleep(LOOP_PERIOD)

    except KeyboardInterrupt:
        print("\nQuitting.")
    except OSError as error:
        print("\nJoystick disconnected (%s)." % error)
    finally:
        mc.stop()
        time.sleep(0.2)
        print("Arm stopped. Final angles:", mc.get_angles())
        os.close(fd)


if __name__ == "__main__":
    main()