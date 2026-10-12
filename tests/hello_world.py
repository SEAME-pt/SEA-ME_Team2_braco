#!/usr/bin/env python3
"""
myCobot 280 Python Hello World Demo
Verifies serial communication, telemetry reading, and LED control.
"""

import argparse
import sys
import time

EXIT_OK = 0
EXIT_COMM_ERROR = 1
EXIT_NO_TELEMETRY = 2

DEFAULT_PORT = "/dev/ttyUSB0"
DEFAULT_BAUD = 115200
READ_ATTEMPTS = 2


def read_angles(mc, attempts=READ_ATTEMPTS):
    """Query joint angles, retrying since the first read after power-on often returns -1."""
    for attempt in range(1, attempts + 1):
        angles = mc.get_angles()
        if isinstance(angles, (list, tuple)) and len(angles) == 6:
            return angles
        print(f"[WARN] Attempt {attempt}/{attempts}: no valid angle data (got {angles!r})")
        time.sleep(0.5)
    return None


def main():
    parser = argparse.ArgumentParser(description="myCobot 280 Hello World Test Script")
    parser.add_argument(
        "--port",
        default=DEFAULT_PORT,
        help=f"Serial port device path (default: {DEFAULT_PORT}; use /dev/ttyTHS1 on Jetson)",
    )
    parser.add_argument(
        "--baud",
        type=int,
        default=DEFAULT_BAUD,
        help=f"Serial baud rate (default: {DEFAULT_BAUD}; use 1000000 on Jetson)",
    )
    args = parser.parse_args()

    try:
        from pymycobot import MyCobot280
    except ImportError:
        print("[ERROR] 'pymycobot' is not installed.")
        print("Install required dependencies with: make venv")
        sys.exit(EXIT_COMM_ERROR)

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
        print(f"  3. If using another adapter, pass port argument: python3 {sys.argv[0]} --port /dev/ttyUSB0")
        sys.exit(EXIT_COMM_ERROR)

    exit_code = EXIT_OK
    try:
        time.sleep(0.5)

        print("[INFO] Powering on servos...")
        mc.power_on()
        time.sleep(0.5)

        print("[INFO] Querying current joint angles...")
        angles = read_angles(mc)

        if angles is None:
            print("[ERROR] No valid angle data returned; check robot power switch and baud rate.")
            print("\nHello-world FAILED: no telemetry received from the robot.")
            exit_code = EXIT_NO_TELEMETRY
        else:
            print(f"[OK] Received joint angles: {angles}")

            print("[INFO] Setting Atom Matrix LED to green (0, 255, 0)...")
            try:
                mc.set_color(0, 255, 0)

            except Exception as e:
                print(f"[WARN] Could not set LED color: {e}")

            print("\n========================================")
            print("Hello, myCobot! Python communication verified successfully.")
            print("========================================")

    except KeyboardInterrupt:
        print("\n[INFO] Script interrupted by user.")
    except Exception as e:
        print(f"\n[ERROR] Communication error: {e}")
        exit_code = EXIT_COMM_ERROR
    finally:
        mc.close()

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
