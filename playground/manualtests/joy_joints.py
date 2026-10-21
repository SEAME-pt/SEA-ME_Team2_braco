#!/usr/bin/env python3
"""
Joystick control of all 6 joints of the myCobot 280 (Jetson Nano).

Continuous motion: while a stick or d-pad direction is held, the joint keeps
moving at constant speed. When it is released, the arm stops.

How it works: on press, the joint is sent towards its limit in that direction
with send_angle(). On release, stop() halts the arm where it is.

Default mapping (Xbox-style layout, change the tables below if yours differs):
    Left stick  X / Y  joint 1 / joint 2
    Right stick Y / X  joint 3 / joint 4
    D-pad       Y / X  joint 5 / joint 6
    A / B              close / open gripper
    Y                  all joints back to 0
    Ctrl+C             quit (the arm is stopped first)

Usage:
    python3 joystick_all_joints.py           # control the arm
    python3 joystick_all_joints.py --debug   # also print every joystick event
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

SPEED = 25  # movement speed while a direction is held (1 to 100)
HOME_SPEED = 25
GRIPPER_SPEED = 50
AXIS_THRESHOLD = 16000  # stick must pass this (max is 32767) to count as pushed
COMMAND_GAP = 0.05  # seconds between two serial commands

# axis number -> (joint 1..6, direction: 1 or -1)
# Run with --debug to see the axis numbers of your controller.
AXIS_TO_JOINT = {
    0: (1, 1),  # left stick X
    1: (2, 1),  # left stick Y
    4: (3, 1),  # right stick Y
    3: (4, 1),  # right stick X
    7: (5, 1),  # d-pad Y
    6: (6, 1),  # d-pad X
}

BUTTON_GRIPPER_CLOSE = 0  # A
BUTTON_GRIPPER_OPEN = 1  # B
BUTTON_HOME = 3  # Y

# Slightly inside the mechanical limits. Index 0 is unused so joint N is LIMITS[N].
LIMITS = [None, 160, 130, 145, 140, 160, 175]

EVENT_FORMAT = 'IhBB'  # time (ms), value, type, number
EVENT_SIZE = struct.calcsize(EVENT_FORMAT)
TYPE_BUTTON = 0x01
TYPE_AXIS = 0x02
TYPE_INIT = 0x80


def read_events(fd):
    """Return every event waiting in the joystick queue (may be empty)."""
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
            continue  # initial state report, not a real press
        events.append((ev_type, number, value))
    return events


def wanted_motion(axis_state):
    """Translate the held axes into {joint: direction}, direction is 1 or -1."""
    wanted = {}
    for axis, (joint, direction) in AXIS_TO_JOINT.items():
        value = axis_state.get(axis, 0)
        if value > AXIS_THRESHOLD:
            wanted[joint] = direction
        elif value < -AXIS_THRESHOLD:
            wanted[joint] = -direction
    return wanted


def quit_on_signal(signum, frame):
    raise KeyboardInterrupt


def main():
    debug = '--debug' in sys.argv

    # Closing the terminal or SSH session must also stop the arm.
    signal.signal(signal.SIGTERM, quit_on_signal)
    signal.signal(signal.SIGHUP, quit_on_signal)

    print('1/2 Opening joystick', JOYSTICK_DEVICE)
    fd = os.open(JOYSTICK_DEVICE, os.O_RDONLY | os.O_NONBLOCK)
    print('    OK')

    print('2/2 Connecting to the arm on', SERIAL_PORT)
    mc = MyCobot280(SERIAL_PORT, BAUD_RATE)
    time.sleep(1)
    mc.power_on()
    time.sleep(0.5)
    print('    Current angles:', mc.get_angles())

    print('Ready. Hold a direction to move, release to stop. Ctrl+C quits.')
    axis_state = {}
    moving = {}  # joint -> direction currently commanded

    try:
        while True:
            for ev_type, number, value in read_events(fd):
                if ev_type == TYPE_AXIS:
                    axis_state[number] = value
                    if debug:
                        print('axis   %d = %d' % (number, value))
                elif ev_type == TYPE_BUTTON:
                    if debug:
                        print('button %d = %d' % (number, value))
                    if value != 1:
                        continue
                    if number == BUTTON_HOME:
                        print('>>> all joints to 0')
                        mc.stop()
                        time.sleep(COMMAND_GAP)
                        mc.send_angles([0, 0, 0, 0, 0, 0], HOME_SPEED)
                        time.sleep(4)
                        read_events(fd)  # discard what piled up during the move
                        axis_state = {}
                        moving = {}
                    elif number == BUTTON_GRIPPER_CLOSE:
                        print('>>> gripper close')
                        mc.set_gripper_state(1, GRIPPER_SPEED)
                    elif number == BUTTON_GRIPPER_OPEN:
                        print('>>> gripper open')
                        mc.set_gripper_state(0, GRIPPER_SPEED)

            wanted = wanted_motion(axis_state)
            if wanted != moving:
                # A joint was released or reversed: stop() halts every joint,
                # so the ones still held are sent again right after.
                released = [j for j in moving if wanted.get(j) != moving[j]]
                if released:
                    print('>>> stop')
                    mc.stop()
                    time.sleep(COMMAND_GAP)
                    moving = {}
                for joint, direction in wanted.items():
                    if joint in moving:
                        continue
                    target = direction * LIMITS[joint]
                    print('>>> send_angle(%d, %d, %d)' % (joint, target, SPEED))
                    mc.send_angle(joint, target, SPEED)
                    time.sleep(COMMAND_GAP)
                moving = dict(wanted)

            time.sleep(0.02)
    except KeyboardInterrupt:
        print('\nQuitting.')
    except OSError as error:
        print('\nJoystick lost (%s).' % error)
    finally:
        # Never leave the arm travelling towards a limit.
        mc.stop()
        time.sleep(0.2)
        print('Arm stopped. Final angles:', mc.get_angles())
        os.close(fd)


if __name__ == '__main__':
    main()
