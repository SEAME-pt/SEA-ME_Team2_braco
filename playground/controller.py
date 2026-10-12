#!/usr/bin/env python3
"""
Joystick control of all 6 joints of the myCobot 280 (Jetson Nano).

Continuous motion with dynamic speed (capped at SPEED_CAP %):
- Analog sticks: speed varies proportionally to stick deflection beyond deadzone.
- Shoulder buttons (L1 / R1): control Joint 6 rotation (tool/gripper).
- Releasing a control immediately stops movement.

Joint 6 protection (J4 drift fix):
- While L1 / R1 is held, the right stick (J4 / J5) is ignored, so stray
  right-stick readings cannot move the wrist during a tool rotation.
- J4 and J5 angles are recorded when J6 starts moving. When J6 stops, any
  drift larger than DRIFT_TOLERANCE_DEG is corrected back automatically.

Mapping:
    Left stick  X / Y  joint 1 / joint 2 (base yaw [inverted] / shoulder pitch)
    Right stick Y / X  joint 4 / joint 5 (vertical = joint 4 / horizontal = joint 5)
    L1 / R1 (6 / 7)    joint 6 (tool/gripper rotation)
    D-pad Y            joint 3 (elbow pitch)
    A / B              close / open gripper
    button 3           all joints back to 0
    Ctrl+C             quit (safely stops the arm)

Usage:
    python3 joystick_all_joints.py           # control the arm
    python3 joystick_all_joints.py --debug   # print joystick events and J4/J5 drift
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

# Speed parameters (pymycobot speed units are 1..100, i.e. percent).
# Change SPEED_CAP to make the whole arm faster or slower.
SPEED_CAP = 30            # no arm motion faster than this
MIN_SPEED = 10            # minimum speed upon exiting deadzone
MAX_SPEED = SPEED_CAP     # maximum speed at full stick deflection
BUMPER_SPEED = 20         # J6 speed on L1 / R1 (kept low: less jerk on the wrist)
HOME_SPEED = SPEED_CAP
HOME_WAIT = 8.0           # seconds to wait for the home move (slower move -> longer wait)
GRIPPER_SPEED = 50        # gripper is a separate actuator; not affected by SPEED_CAP
SPEED_STEP = 5            # quantization step for stick speeds (10, 15, 20, 25, 30)

# J4 / J5 drift correction after a J6 move
DRIFT_TOLERANCE_DEG = 1.0  # ignore drift smaller than this (servo noise / resolution)
RESTORE_SPEED = 15         # speed used to put J4 / J5 back

AXIS_THRESHOLD = 16000    # deadzone (axis range is -32767 to 32767)
COMMAND_GAP = 0.05        # safety delay between serial commands

# axis number -> (joint 1..6, direction: 1 or -1)
AXIS_TO_JOINT = {
    0: (1, -1),  # left stick X  -> joint 1 (base yaw) [inverted]
    1: (2, 1),   # left stick Y  -> joint 2 (shoulder pitch)
    3: (4, -1),  # right stick vertical   -> joint 4 (up / down)
    4: (5, -1),  # right stick horizontal -> joint 5 (pre-gripper wrist: left / right)
    7: (3, 1),   # d-pad Y       -> joint 3 (elbow)
}

# Axes ignored while L1 / R1 is held (the right stick: J4 and J5)
LOCKED_DURING_J6 = {3, 4}

# Controller buttons
BUTTON_GRIPPER_CLOSE = 0  # A
BUTTON_GRIPPER_OPEN = 1   # B
BUTTON_HOME = 3           # home (check with --debug which physical button this is)
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


def read_angles(mc):
    """Return the 6 current joint angles, or None if the reading is invalid."""
    try:
        angles = mc.get_angles()
    except Exception:
        return None
    if (isinstance(angles, list) and len(angles) == 6
            and all(isinstance(a, (int, float)) for a in angles)):
        return angles
    return None


def calculate_speed(raw_value):
    """Calculate proportional speed based on stick deflection beyond deadzone."""
    magnitude = abs(raw_value)
    if magnitude <= AXIS_THRESHOLD:
        return 0
    # Normalized ratio between 0.0 and 1.0 above deadzone
    ratio = (magnitude - AXIS_THRESHOLD) / (32767.0 - AXIS_THRESHOLD)
    raw_speed = MIN_SPEED + ratio * (MAX_SPEED - MIN_SPEED)
    # Quantize to avoid flooding the serial bus
    quantized_speed = int(round(raw_speed / SPEED_STEP) * SPEED_STEP)
    return max(MIN_SPEED, min(SPEED_CAP, quantized_speed))


def wanted_motion(axis_state, button_state):
    """Return desired {joint: (direction, speed)} for axes and L1/R1 buttons."""
    wanted = {}

    l1_active = button_state.get(BUTTON_L1, 0) == 1
    r1_active = button_state.get(BUTTON_R1, 0) == 1
    bumper_active = l1_active or r1_active

    # Configured analog axes (right stick locked while a bumper is held)
    for axis, (joint, direction) in AXIS_TO_JOINT.items():
        if bumper_active and axis in LOCKED_DURING_J6:
            continue
        value = axis_state.get(axis, 0)
        spd = calculate_speed(value)
        if spd > 0:
            dir_factor = direction if value > 0 else -direction
            wanted[joint] = (dir_factor, spd)

    # Joint 6 controlled by L1 and R1 shoulder buttons
    if l1_active and not r1_active:
        wanted[6] = (-1, BUMPER_SPEED)
    elif r1_active and not l1_active:
        wanted[6] = (1, BUMPER_SPEED)

    return wanted


def restore_wrist(mc, snapshot, wanted, debug):
    """After a J6 move, put J4 / J5 back where they were if they drifted."""
    time.sleep(0.2)  # let the arm settle before measuring
    now = read_angles(mc)
    if now is None:
        print("!!! could not read angles, J4/J5 drift not checked")
        return
    for joint, before in snapshot.items():
        drift = now[joint - 1] - before
        if debug or abs(drift) > DRIFT_TOLERANCE_DEG:
            print("    J%d drift during J6 move: %+.1f deg" % (joint, drift))
        if abs(drift) > DRIFT_TOLERANCE_DEG and joint not in wanted:
            print(">>> restoring J%d to %.1f" % (joint, before))
            mc.send_angle(joint, round(before, 1), RESTORE_SPEED)
            time.sleep(COMMAND_GAP)


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

    print("Ready. Speed cap %d %%, J6 at %d %%. Ctrl+C quits." % (SPEED_CAP, BUMPER_SPEED))
    axis_state = {}
    button_state = {}
    moving = {}          # joint -> (direction, speed) currently in execution
    wrist_snapshot = None  # {4: angle, 5: angle} recorded when J6 starts moving

    try:
        while True:
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
                        if number == BUTTON_HOME:
                            print(">>> all joints to 0")
                            mc.stop()
                            time.sleep(COMMAND_GAP)
                            mc.send_angles([0, 0, 0, 0, 0, 0], HOME_SPEED)
                            time.sleep(HOME_WAIT)
                            read_events(fd)
                            axis_state.clear()
                            button_state.clear()
                            moving.clear()
                            wrist_snapshot = None
                        elif number == BUTTON_GRIPPER_CLOSE:
                            print(">>> close gripper")
                            mc.set_gripper_state(1, GRIPPER_SPEED)
                        elif number == BUTTON_GRIPPER_OPEN:
                            print(">>> open gripper")
                            mc.set_gripper_state(0, GRIPPER_SPEED)

            wanted = wanted_motion(axis_state, button_state)

            if wanted != moving:
                # Detect joints that were released or reversed direction
                stopped_or_reversed = [
                    j for j, (cur_dir, _) in moving.items()
                    if j not in wanted or wanted[j][0] != cur_dir
                ]

                # If any joint stopped or reversed, execute stop()
                if stopped_or_reversed:
                    print(">>> stop")
                    mc.stop()
                    time.sleep(COMMAND_GAP)
                    moving.clear()

                # J6 finished: check and correct J4 / J5 drift
                if 6 not in wanted and wrist_snapshot is not None:
                    restore_wrist(mc, wrist_snapshot, wanted, debug)
                    wrist_snapshot = None

                # J6 starting: record J4 / J5 before it moves
                if 6 in wanted and wrist_snapshot is None:
                    angles = read_angles(mc)
                    if angles is not None:
                        wrist_snapshot = {4: angles[3], 5: angles[4]}

                # Send commands to newly engaged joints or joints with updated speed steps
                for joint, (direction, spd) in wanted.items():
                    if joint in moving:
                        if moving[joint][1] != spd:
                            target = direction * LIMITS[joint]
                            mc.send_angle(joint, target, spd)
                            moving[joint] = (direction, spd)
                            time.sleep(COMMAND_GAP)
                    else:
                        target = direction * LIMITS[joint]
                        print(">>> send_angle(%d, %d, vel=%d)" % (joint, target, spd))
                        mc.send_angle(joint, target, spd)
                        moving[joint] = (direction, spd)
                        time.sleep(COMMAND_GAP)

            time.sleep(0.02)
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