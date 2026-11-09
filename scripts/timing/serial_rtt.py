#!/usr/bin/env python3
"""Serial round trip of one joint read (getAngles, 0x20) to the myCobot 280 Atom, without ROS.

Issue #90, measurement 1. The port has a single owner: run it with arm_driver STOPPED.

    python3 serial_rtt.py                       # /dev/ttyTHS1, 1 000 000 baud, 1000 reads
    python3 serial_rtt.py -n 2000 --gap-ms 0    # back to back

Prints mean, p50, p99 and max in ms, and the reads that got no valid reply.
"""

import argparse
import math
import time

import serial

REQUEST = bytes([0xFE, 0xFE, 0x02, 0x20, 0xFA])  # FE FE <len=2> <cmd> FA
REPLY_HEAD = bytes([0xFE, 0xFE, 0x0E, 0x20])  # len = 12 data bytes + 2
REPLY_LEN = 16
REPLY_TIMEOUT_S = 0.5  # as mycobot.cpp (REPLY_TIMEOUT)


def percentile(sorted_ms, p):
    return sorted_ms[max(0, math.ceil(p * len(sorted_ms)) - 1)]


def read_reply(port):
    """True once a complete getAngles reply has arrived, False after REPLY_TIMEOUT_S."""
    deadline = time.perf_counter() + REPLY_TIMEOUT_S
    buf = bytearray()
    while time.perf_counter() < deadline:
        buf += port.read(port.in_waiting or 1)
        i = buf.find(REPLY_HEAD)
        if i >= 0 and len(buf) >= i + REPLY_LEN:
            return buf[i + REPLY_LEN - 1] == 0xFA
    return False


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('--port', default='/dev/ttyTHS1')
    ap.add_argument('--baud', type=int, default=1_000_000)
    ap.add_argument('-n', type=int, default=1000, help='number of reads')
    ap.add_argument('--gap-ms', type=float, default=5.0, help='pause between reads')
    args = ap.parse_args()

    samples, failures = [], 0
    with serial.Serial(args.port, args.baud, timeout=0.01) as port:
        time.sleep(1.0)  # the Atom needs a moment after the port opens (as zero.cpp / park.cpp)
        for _ in range(args.n):
            port.reset_input_buffer()  # tcflush(TCIFLUSH), as query() does
            t0 = time.perf_counter()
            port.write(REQUEST)
            port.flush()  # tcdrain: the request has left the UART
            ok = read_reply(port)
            dt_ms = (time.perf_counter() - t0) * 1000.0
            if ok:
                samples.append(dt_ms)
            else:
                failures += 1
            time.sleep(args.gap_ms / 1000.0)

    if not samples:
        print(f'no valid reply in {args.n} reads: is arm_driver stopped and the Atom on?')
        return
    s = sorted(samples)
    mean = sum(s) / len(s)
    print(f'{args.port} @ {args.baud} baud, {args.n} reads, gap {args.gap_ms} ms')
    print('| reads | mean ms | p50 ms | p99 ms | max ms | failed |')
    print('|---|---|---|---|---|---|')
    print(
        f'| {len(s)} | {mean:.2f} | {percentile(s, 0.50):.2f} | {percentile(s, 0.99):.2f} '
        f'| {s[-1]:.2f} | {failures} |'
    )


if __name__ == '__main__':
    main()
