#!/usr/bin/env python3
"""Few long moves instead of many micro-moves: is the arm smoother, and is the queue mode safe?

Proposal (after #93): MoveIt's path has only a few vertices, but today it reaches the arm as
hundreds of micro-targets (JTC 100 Hz -> BracoSystem -> arm_driver ~14 Hz), and the Atom replans
on every one. This script measures, straight on the serial port, what the firmware does with
fewer, longer moves, in its two fresh modes:

  fresh mode 1  the latest command interrupts the current move (what arm_driver relies on today)
  fresh mode 0  commands queue: the next starts when the previous one has finished

Tests (run `stop` first: if it fails, mode 0 is ruled out and `smooth` skips it):

  stop    mode 0: queue 3 waypoints, send STOP during the first one, watch the arm for 3 s.
          PASS only if the arm stays where it stopped (STOP also empties the queue).
  smooth  the same long move (default J1 +40 deg and J2 +20 deg together, then back) four ways:
            stream     a target every ~70 ms along a straight line in time, as the system does now
            single     one SEND_ANGLES to the end, the firmware does the whole move
            wp-mode1   3 waypoints in mode 1, each sent when the previous one has arrived
            wp-mode0   3 waypoints queued at once in mode 0 (skipped if `stop` did not pass)
          For each: cv of the velocity (std / mean, lower = smoother), stops on the way, time.

The arm_driver must be STOPPED (this script owns the port): make down, then run it in a
throw-away real container. The arm must be powered on, at zero, with room for the move.
It always puts the fresh mode back to 1 and reads it back, also on errors and Ctrl-C.
Nothing is sent without --yes.

    make down
    cd docker && COMPOSE_PROFILES=jetson docker compose run --rm real bash
    cd /opt/braco/scripts/timing && python3 waypoint_test.py stop --yes
    python3 waypoint_test.py smooth --yes --out /ws/bags/timing/waypoints.jsonl
    exit; cd .. && make up && make web
"""

import argparse
import json
import statistics as st
import sys
import time

from serial_rtt import open_exclusive

# Protocol (docs/hardware/protocol_commands.md; fresh mode from the Elephant protocol)
POWER_ON, GET_ANGLES, SEND_ANGLES, STOP, IS_MOVING = 0x10, 0x20, 0x22, 0x29, 0x2B
SET_FRESH_MODE, GET_FRESH_MODE = 0x16, 0x17
# JOINT_LIMITS of mycobot.hpp (degrees) minus a margin: no target goes past them
LIMITS = [(-166, 166), (-138, 138), (-149, 149), (-149, 149), (-153, 158), (-178, 178)]
MARGIN_DEG = 5.0
COMMAND_GAP_S = 0.05  # arm_driver's default command_gap_ms: never faster than the system


class Atom:
    """The few frames this test needs, over an exclusive port."""

    def __init__(self, port):
        self.port = port

    def send(self, cmd, data=b''):
        self.port.write(bytes([0xFE, 0xFE, len(data) + 2, cmd]) + bytes(data) + b'\xfa')
        self.port.flush()

    def query(self, cmd, data=b'', timeout=0.3):
        """The data bytes of the reply to `cmd`, or None."""
        self.port.reset_input_buffer()
        self.send(cmd, data)
        buf = bytearray()
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            buf += self.port.read(self.port.in_waiting or 1)
            i = buf.find(b'\xfe\xfe')
            while i >= 0 and len(buf) >= i + 4:
                n = buf[i + 2]
                if len(buf) < i + 3 + n:
                    break
                frame = buf[i : i + 3 + n]
                if frame[-1] == 0xFA and frame[3] == cmd:
                    return bytes(frame[4:-1])
                i = buf.find(b'\xfe\xfe', i + 1)
        return None

    def angles(self):
        d = self.query(GET_ANGLES)
        if not d or len(d) != 12:
            return None
        return [int.from_bytes(d[k : k + 2], 'big', signed=True) / 100.0 for k in range(0, 12, 2)]

    def send_angles(self, angles, speed):
        for j, a in enumerate(angles):
            lo, hi = LIMITS[j]
            if not lo + MARGIN_DEG <= a <= hi - MARGIN_DEG:
                raise RuntimeError(f'J{j + 1} target {a:.1f} is outside the limits')
        data = b''.join(int(round(a * 100)).to_bytes(2, 'big', signed=True) for a in angles)
        self.send(SEND_ANGLES, data + bytes([max(1, min(100, speed))]))

    def moving(self):
        d = self.query(IS_MOVING)
        return bool(d and d[0])

    def fresh_mode(self):
        d = self.query(GET_FRESH_MODE)
        return d[0] if d else None

    def set_fresh_mode(self, mode):
        self.send(SET_FRESH_MODE, bytes([mode]))
        time.sleep(0.1)
        return self.fresh_mode() == mode


def record(atom, until, samples, period=0.02):
    """Read the angles until `until()` is true; append (t, angles) to samples."""
    while not until():
        a = atom.angles()
        if a:
            samples.append((time.monotonic(), a))
        time.sleep(period)


def lerp(a, b, f):
    return [x + (y - x) * f for x, y in zip(a, b)]


def metrics(samples, start, goal, joint):
    """cv of the velocity of `joint` while moving, stops on the way, time to arrive."""
    j = joint - 1
    total = abs(goal[j] - start[j])
    moving = [
        (t, a[j])
        for t, a in samples
        if 1.0 < abs(a[j] - start[j]) and abs(goal[j] - a[j]) > 2.0 and total > 3.0
    ]
    if len(moving) < 4:
        return {'cv_v': None, 'stops': None, 'v_mean': None}
    v = [abs((b[1] - a[1]) / (b[0] - a[0])) for a, b in zip(moving, moving[1:]) if b[0] > a[0]]
    vm = abs(moving[-1][1] - moving[0][1]) / (moving[-1][0] - moving[0][0])
    # A stop: velocity under 15 % of the mean for 2+ consecutive samples
    stops, run = 0, 0
    for x in v:
        run = run + 1 if x < 0.15 * vm else 0
        stops += run == 2
    return {'cv_v': round(st.pstdev(v) / vm, 3), 'stops': stops, 'v_mean': round(vm, 1)}


def arrived(atom, goal, tol=2.0):
    a = atom.angles()
    return a is not None and max(abs(x - y) for x, y in zip(a, goal)) <= tol


def run_way(atom, way, start, goal, speed, waypoints):
    """One move start -> goal; returns the samples (t, angles)."""
    samples = []
    t_end = time.monotonic() + 30.0

    def timed_out():
        return time.monotonic() > t_end

    def done():
        return timed_out() or (not atom.moving() and arrived(atom, goal))

    if way == 'single':
        atom.send_angles(goal, speed)
        record(atom, done, samples)
    elif way == 'stream':
        # Straight line in time at the speed's nominal velocity, a target per serial cycle
        dist = max(abs(g - s) for g, s in zip(goal, start))
        duration = dist / (speed * 1.52)
        t0 = time.monotonic()
        while not timed_out():
            f = min(1.0, (time.monotonic() - t0) / duration)
            atom.send_angles(lerp(start, goal, f), speed)
            time.sleep(COMMAND_GAP_S)
            a = atom.angles()
            if a:
                samples.append((time.monotonic(), a))
            if f >= 1.0:
                break
        record(atom, done, samples)
    elif way == 'wp-mode1':
        for wp in waypoints:
            atom.send_angles(wp, speed)
            time.sleep(COMMAND_GAP_S)
            record(atom, lambda wp=wp: timed_out() or arrived(atom, wp, 3.0), samples)
        record(atom, done, samples)
    elif way == 'wp-mode0':
        if not atom.set_fresh_mode(0):
            raise RuntimeError('could not set fresh mode 0')
        try:
            for wp in waypoints:
                atom.send_angles(wp, speed)
                time.sleep(COMMAND_GAP_S)
            record(atom, done, samples)
        finally:
            restore_mode(atom)
    if timed_out():
        atom.send(STOP)
        raise RuntimeError(f'{way}: did not arrive within 30 s, STOP sent')
    return samples


def restore_mode(atom):
    """Fresh mode back to 1 (what arm_driver relies on), read back. Exit loudly if it fails."""
    for _ in range(3):
        if atom.set_fresh_mode(1):
            return
    print('!!! fresh mode is NOT back to 1: set it before starting arm_driver', file=sys.stderr)
    sys.exit(2)


def test_stop(atom, start, args):
    goal = list(start)
    goal[0] += args.j1
    wps = [lerp(start, goal, k / args.waypoints) for k in range(1, args.waypoints + 1)]
    if not atom.set_fresh_mode(0):
        raise RuntimeError('could not set fresh mode 0')
    try:
        for wp in wps:
            atom.send_angles(wp, args.speed)
            time.sleep(COMMAND_GAP_S)
        time.sleep(0.4)  # inside the first segment
        # Read right BEFORE the STOP: read after it, a queue that survived the STOP would already
        # have moved the arm on, and the test would pass wrongly
        at_stop = atom.angles()
        if args.clear == 'mode1':
            # Does leaving fresh mode 0 empty the queue? Then STOP stops the current move
            atom.send(SET_FRESH_MODE, bytes([1]))
            time.sleep(0.02)
        atom.send(STOP)
        time.sleep(3.0)
        after = atom.angles()
        still_moving = atom.moving()
    finally:
        restore_mode(atom)
    # The arm coasts a little while braking: up to 3 deg past the STOP point is a stop
    drift = abs(after[0] - at_stop[0]) if at_stop and after else None
    ok = drift is not None and drift < 3.0 and not still_moving
    result = {
        'test': 'stop',
        'clear': args.clear,
        'j1_at_stop': at_stop and round(at_stop[0], 1),
        'j1_after_3s': after and round(after[0], 1),
        'drift_deg': drift if drift is None else round(drift, 1),
        'moving_after_3s': still_moving,
        'pass': ok,
    }
    print(json.dumps(result))
    print('PASS: STOP empties the queue' if ok else 'FAIL: the arm went on after STOP: no mode 0')
    return result


def test_smooth(atom, start, args):
    goal = list(start)
    goal[0] += args.j1
    goal[1] += args.j2
    ways = ['stream', 'single', 'wp-mode1'] + (['wp-mode0'] if args.mode0 else [])
    results = []
    for way in ways:
        for a, b in ((start, goal), (goal, start)):
            wps = [lerp(a, b, k / args.waypoints) for k in range(1, args.waypoints + 1)]
            t0 = time.monotonic()
            samples = run_way(atom, way, a, b, args.speed, wps)
            r = {'test': 'smooth', 'way': way, 'dir': 'out' if a is start else 'back'}
            r.update(
                {f'j{j}_{k}': v for j in (1, 2) for k, v in metrics(samples, a, b, j).items()}
            )
            r['time_s'] = round(time.monotonic() - t0, 2)
            print(json.dumps(r))
            results.append(r)
            time.sleep(1.0)
    return results


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('test', choices=['stop', 'smooth'])
    ap.add_argument('--yes', action='store_true', help='really move the arm')
    ap.add_argument('--port', default='/dev/ttyTHS1')
    ap.add_argument('--baud', type=int, default=1_000_000)
    ap.add_argument('--speed', type=int, default=15, help='firmware speed 1..100')
    ap.add_argument('--j1', type=float, default=40.0, help='J1 travel, degrees')
    ap.add_argument('--j2', type=float, default=20.0, help='J2 travel, degrees (smooth)')
    ap.add_argument(
        '--mode0', action='store_true', help='smooth: include wp-mode0 (after stop PASS)'
    )
    ap.add_argument('--waypoints', type=int, default=3, help='waypoints per move (stop, smooth)')
    ap.add_argument(
        '--clear',
        choices=['stop', 'mode1'],
        default='stop',
        help='stop: STOP alone, or fresh mode 1 then STOP (does that empty the queue?)',
    )
    ap.add_argument('--out', help='JSON lines file to append the results to')
    args = ap.parse_args()
    if not args.yes:
        sys.exit('this moves the arm: add --yes (someone at the arm, STOP ready)')

    with open_exclusive(args.port, args.baud) as port:
        time.sleep(1.0)  # the Atom needs a moment after the port opens
        atom = Atom(port)
        mode = atom.fresh_mode()
        print(f'fresh mode at start: {mode}')
        atom.send(POWER_ON)
        time.sleep(0.5)
        start = atom.angles()
        if not start:
            sys.exit('no angle reading: is the arm powered?')
        print('start pose:', ' '.join(f'{a:.1f}' for a in start))
        try:
            results = test_stop(atom, start, args) if args.test == 'stop' else None
            if args.test == 'smooth':
                results = test_smooth(atom, start, args)
                atom.send_angles(start, args.speed)  # back where it began
                record(atom, lambda: arrived(atom, start) or not atom.moving(), [])
        except KeyboardInterrupt:
            atom.send(STOP)
            print('interrupted: STOP sent')
            results = None
        finally:
            restore_mode(atom)
            print(f'fresh mode at end: {atom.fresh_mode()}')
        if args.out and results:
            with open(args.out, 'a') as f:
                for r in results if isinstance(results, list) else [results]:
                    f.write(json.dumps(r) + '\n')


if __name__ == '__main__':
    main()
