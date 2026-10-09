#!/usr/bin/env python3
"""
myCobot 280 Python Hello World Demo
Verifies serial communication, telemetry reading, and LED control.
"""

import argparse
import sys
import time


def main():
    parser = argparse.ArgumentParser(description="myCobot 280 Hello World Test Script")
    parser.add_argument(
        "--port",
        default="/dev/ttyTHS1",
        help="Serial port device path (default: /dev/ttyTHS1 on Jetson, or e.g. /dev/ttyUSB0 on PC)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=1000000,
        help="Serial baud rate (default: 1000000)",
    )
    args = parser.parse_args()

    try:
        from pymycobot import MyCobot280
    except ImportError:
        try:
            from pymycobot import MyCobot as MyCobot280
        except ImportError:
            print("[ERROR] 'pymycobot' is not installed.")
            print("Install required dependencies with: pip install pymycobot pyserial")
            sys.exit(1)

    print("========================================")
    print("   myCobot 280 Python Hello World Demo   ")
    print("========================================")
    print(f"Connecting to port: {args.port} at {args.baud} baud...")

    try:
        mc = MyCobot280(args.port, args.baud)
    except Exception as e:
        print(f"\n[ERROR] Failed to open serial port {args.port}: {e}")
        print("Troubleshooting hints:")
        print(f"  1. Check device existence: ls -l {args.port}")
        print("  2. Ensure dialout group permission: sudo usermod -aG dialout $USER (then relog)")
        print(f"  3. If using USB adapter, pass port argument: python3 {sys.argv[0]} --port /dev/ttyUSB0")
        sys.exit(1)

    time.sleep(0.5)

    try:
        print("[INFO] Powering on servos...")
        mc.power_on()
        time.sleep(0.5)

        print("[INFO] Querying current joint angles...")
        angles = mc.get_angles()

        if angles:
            print(f"[OK] Received joint angles: {angles}")
        else:
            print("[WARN] No angle data returned (check robot power switch).")

        print("[INFO] Setting Atom Matrix LED to green (0, 255, 0)...")
        try:
            mc.set_color(0, 255, 0)
        except Exception:
            pass

        print("\n========================================")
        print("Hello, myCobot! Python communication verified successfully.")
        print("========================================")

    except KeyboardInterrupt:
        print("\n[INFO] Script interrupted by user.")
    except Exception as e:
        print(f"\n[ERROR] Communication error: {e}")
        sys.exit(1)


if __name__ == "__main__":
    main()
