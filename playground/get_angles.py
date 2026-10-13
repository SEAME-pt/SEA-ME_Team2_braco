#!/usr/bin/env python3
"""
Release the joints of a myCobot 280 Jetson Nano so you can move the arm
by hand, then print the joint angles every time you press Enter.

Requirements:
    pip install pymycobot

Usage:
    python3 get_mycobot_angles.py
    python3 get_mycobot_angles.py --coords      # also print X, Y, Z, RX, RY, RZ

Controls:
    Enter      -> read and print the current angles
    q + Enter  -> lock the joints and quit
"""

import argparse
import sys
import time

# Newer pymycobot versions provide MyCobot280; older ones only have MyCobot.
try:
    from pymycobot import MyCobot280 as MyCobot
except ImportError:
    from pymycobot.mycobot import MyCobot

DEFAULT_PORT = "/dev/ttyTHS1"
DEFAULT_BAUD = 1000000


def read_value(getter, retries=5):
    """Call getter() until it returns a list of 6 numbers, or give up."""
    for _ in range(retries):
        value = getter()
        if isinstance(value, list) and len(value) == 6:
            return value
        time.sleep(0.1)
    return None


def main():
    parser = argparse.ArgumentParser(description="Release joints and read angles on Enter")
    parser.add_argument("--port", default=DEFAULT_PORT, help="Serial port")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Baud rate")
    parser.add_argument("--coords", action="store_true",
                        help="Also print Cartesian coordinates")
    args = parser.parse_args()

    try:
        mc = MyCobot(args.port, args.baud)
    except Exception as e:
        print(f"Could not open {args.port}: {e}")
        print("Tip: check the port and run with sudo or add your user to the 'dialout' group.")
        sys.exit(1)

    time.sleep(0.5)

    input("Hold the arm with your hand, then press Enter to release the joints...")
    mc.release_all_servos()
    time.sleep(0.5)
    print("Joints released. Move the arm by hand.")
    print("Press Enter to read the angles, or type 'q' and press Enter to quit.\n")

    saved = []

    try:
        while True:
            cmd = input("> ").strip().lower()
            if cmd == "q":
                break

            angles = read_value(mc.get_angles)
            if angles is None:
                print("Failed to read angles (is the robot powered on?)")
                continue

            saved.append(angles)
            joints = "  ".join(f"J{i + 1}: {a:8.2f}°" for i, a in enumerate(angles))
            print(f"[{len(saved)}] Angles -> {joints}")

            if args.coords:
                coords = read_value(mc.get_coords)
                if coords is not None:
                    labels = ["X", "Y", "Z", "RX", "RY", "RZ"]
                    values = "  ".join(f"{l}: {v:8.2f}" for l, v in zip(labels, coords))
                    print(f"    Coords -> {values}")
                else:
                    print("    Failed to read coordinates")
    except (KeyboardInterrupt, EOFError):
        print()

    # Lock the joints again so the arm holds its position.
    input("Hold the arm, then press Enter to lock the joints...")
    mc.power_on()
    time.sleep(0.5)
    print("Joints locked.")

    if saved:
        print("\nAll recorded positions (ready to paste into mc.send_angles):")
        for i, a in enumerate(saved, 1):
            print(f"{i}: {[round(x, 2) for x in a]}")


if __name__ == "__main__":
    main()
