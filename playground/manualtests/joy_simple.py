#!/usr/bin/env python3
"""
Simplest possible joystick test for the myCobot 280 (Jetson Nano).

It does not depend on the button layout of your controller:
    push ANY stick or d-pad direction -> joint 1 steps 10 degrees left or right
    press ANY button                  -> joint 1 goes back to 0
    Ctrl+C                            -> quit

Every joystick event is printed, so you can see what the script receives.

Usage:
    python3 joystick_simple_test.py
"""

import os
import struct
import time

from pymycobot import MyCobot280

SERIAL_PORT = '/dev/ttyTHS1'
BAUD_RATE = 1000000
JOYSTICK_DEVICE = '/dev/input/js0'

JOINT = 1  # joint moved by the test
STEP_DEG = 10  # degrees per push
LIMIT_DEG = 150  # keeps joint 1 away from its +-168 limit
SPEED = 25  # same speed as the working base script

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


def main():
    print('1/3 Opening joystick', JOYSTICK_DEVICE)
    fd = os.open(JOYSTICK_DEVICE, os.O_RDONLY | os.O_NONBLOCK)
    print('    OK')

    print('2/3 Connecting to the arm on', SERIAL_PORT)
    mc = MyCobot280(SERIAL_PORT, BAUD_RATE)
    time.sleep(1)
    mc.power_on()
    time.sleep(0.5)
    print('    Current angles:', mc.get_angles())

    print('3/3 Moving joint %d to 0' % JOINT)
    mc.send_angle(JOINT, 0, SPEED)
    time.sleep(3)
    angle = 0

    print('Ready. Push a stick to move, press a button to go back to 0.')
    try:
        while True:
            new_angle = angle
            for ev_type, number, value in read_events(fd):
                if ev_type == TYPE_AXIS:
                    print('axis   %d = %d' % (number, value))
                    if value > 16000:
                        new_angle = angle + STEP_DEG
                    elif value < -16000:
                        new_angle = angle - STEP_DEG
                elif ev_type == TYPE_BUTTON:
                    print('button %d = %d' % (number, value))
                    if value == 1:
                        new_angle = 0

            new_angle = max(-LIMIT_DEG, min(LIMIT_DEG, new_angle))
            if new_angle != angle:
                angle = new_angle
                print('>>> send_angle(%d, %d, %d)' % (JOINT, angle, SPEED))
                mc.send_angle(JOINT, angle, SPEED)
                time.sleep(1)
                read_events(fd)  # discard what piled up during the move

            time.sleep(0.02)
    except KeyboardInterrupt:
        print('\nBye. Final angles:', mc.get_angles())
    finally:
        os.close(fd)


if __name__ == '__main__':
    main()
