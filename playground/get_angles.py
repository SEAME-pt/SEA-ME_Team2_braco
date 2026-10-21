#!/usr/bin/env python3
"""
Toggle the joints of a myCobot 280 Jetson Nano with Enter so you can
pose the arm by hand: the 1st Enter releases the joints, the 2nd Enter
locks them and prints the current angles, the 3rd releases again, etc.

Requirements:
    pip install pymycobot

Usage:
    python3 get_mycobot_angles.py
    python3 get_mycobot_angles.py --coords      # also print X, Y, Z, RX, RY, RZ

Controls:
    Enter (arm released) -> lock the joints and print the current angles
    Enter (arm locked)   -> release the joints to move to the next pose
    q + Enter            -> lock the joints (if needed) and quit
"""

import argparse
import sys
import time

# Newer pymycobot versions provide MyCobot280; older ones only have MyCobot.
try:
    from pymycobot import MyCobot280 as MyCobot
except ImportError:
    from pymycobot.mycobot import MyCobot

DEFAULT_PORT = '/dev/ttyTHS1'
DEFAULT_BAUD = 1000000


def read_value(getter, retries=5):
    """Call getter() until it returns a list of 6 numbers, or give up."""
    for _ in range(retries):
        value = getter()
        if isinstance(value, list) and len(value) == 6:
            return value
        time.sleep(0.1)
    return None


def lock(mc):
    mc.power_on()
    time.sleep(0.5)


def release(mc):
    mc.release_all_servos()
    time.sleep(0.5)


def main():
    parser = argparse.ArgumentParser(description='Release joints and read angles on Enter')
    parser.add_argument('--port', default=DEFAULT_PORT, help='Serial port')
    parser.add_argument('--baud', type=int, default=DEFAULT_BAUD, help='Baud rate')
    parser.add_argument('--coords', action='store_true', help='Also print Cartesian coordinates')
    args = parser.parse_args()

    try:
        mc = MyCobot(args.port, args.baud)
    except Exception as e:
        print(f'Could not open {args.port}: {e}')
        print("Tip: check the port and run with sudo or add your user to the 'dialout' group.")
        sys.exit(1)

    time.sleep(0.5)

    # Start locked: 1st Enter releases, 2nd Enter locks and reads, and so on.
    lock(mc)
    locked = True
    print('Joints locked.')
    print('Enter toggles: release -> lock and read -> release -> ...')
    print("Type 'q' and press Enter to quit.\n")

    saved = []

    try:
        while True:
            if locked:
                prompt = '[locked] Hold the arm, then Enter to release (q to quit) > '
            else:
                prompt = '[released] Enter to lock and read angles (q to quit) > '

            cmd = input(prompt).strip().lower()
            if cmd == 'q':
                break

            if locked:
                release(mc)
                locked = False
                print('Joints released. Move the arm to the next position.\n')
                continue

            # Lock first so the arm holds the pose while we read it.
            lock(mc)
            locked = True

            angles = read_value(mc.get_angles)
            if angles is None:
                print('Failed to read angles (is the robot powered on?)')
                continue

            saved.append(angles)
            joints = '  '.join(f'J{i + 1}: {a:8.2f}°' for i, a in enumerate(angles))
            print(f'[{len(saved)}] Angles -> {joints}')

            if args.coords:
                coords = read_value(mc.get_coords)
                if coords is not None:
                    labels = ['X', 'Y', 'Z', 'RX', 'RY', 'RZ']
                    values = '  '.join(f'{l}: {v:8.2f}' for l, v in zip(labels, coords))
                    print(f'    Coords -> {values}')
                else:
                    print('    Failed to read coordinates')
    except (KeyboardInterrupt, EOFError):
        print()

    # Make sure the arm ends up locked.
    if not locked:
        input('Hold the arm, then press Enter to lock the joints...')
        lock(mc)

    print('Joints locked.')

    if saved:
        print('\nAll recorded positions (ready to paste into mc.send_angles):')
        for i, a in enumerate(saved, 1):
            print(f'{i}: {[round(x, 2) for x in a]}')


if __name__ == '__main__':
    main()
