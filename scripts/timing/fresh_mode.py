#!/usr/bin/env python3
"""Read or set the Atom's fresh mode (1: the latest command wins, 0: commands queue).

arm_driver never sets it: whatever this leaves is what the system runs with. Run it with
arm_driver STOPPED (it owns the port), in a throw-away real container. Setting it is read back.

    python3 fresh_mode.py           # print it
    python3 fresh_mode.py --set 0   # queue mode (STOP no longer empties the queue: see ADR-0006)
    python3 fresh_mode.py --set 1   # back to what arm_driver relies on
"""

import argparse
import sys
import time

from serial_rtt import open_exclusive
from waypoint_test import Atom


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--set', type=int, choices=[0, 1], dest='mode')
    ap.add_argument('--port', default='/dev/ttyTHS1')
    ap.add_argument('--baud', type=int, default=1_000_000)
    args = ap.parse_args()
    with open_exclusive(args.port, args.baud) as port:
        time.sleep(1.0)  # the Atom needs a moment after the port opens
        atom = Atom(port)
        print(f'fresh mode: {atom.fresh_mode()}')
        if args.mode is not None:
            ok = atom.set_fresh_mode(args.mode)
            print(f'fresh mode now: {atom.fresh_mode()}')
            if not ok:
                sys.exit(f'could not set fresh mode {args.mode}')


if __name__ == '__main__':
    main()
