#!/usr/bin/env python3
"""Jitter plots of js_rate.py --raw runs: a histogram and a time series (issue #93).

    python3 plot_jitter.py --requested 20 --out <folder> \
        --run default=<default.csv> --run fifo=<fifo.csv>

Prints the statistics of each run (the same as js_rate.py) and writes histogram.png and
timeseries.png to --out.
"""

import argparse
import os

from stats import interval_stats, intervals_ms, load_stamps


def parse_run(text):
    """'label=path' -> (label, path)."""
    label, sep, path = text.partition('=')
    if not sep or not label or not path:
        raise argparse.ArgumentTypeError(f'expected label=path, got {text!r}')
    return label, path


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument(
        '--run',
        type=parse_run,
        action='append',
        required=True,
        help='label=path of a js_rate.py --raw CSV (repeat for each run)',
    )
    ap.add_argument('--requested', type=float, required=True, help='state_rate_hz of the runs')
    ap.add_argument('--out', default='.', help='folder for the PNG files')
    args = ap.parse_args()

    runs = []
    for label, path in args.run:
        stamps = load_stamps(path)
        s = interval_stats(stamps, args.requested)
        print(
            f'{label}: {len(stamps)} msgs, {s["achieved_hz"]:.1f} Hz, mean {s["mean_ms"]:.1f}, '
            f'p50 {s["p50_ms"]:.1f}, p99 {s["p99_ms"]:.1f}, max {s["max_ms"]:.1f} ms, '
            f'late {s["late_pct"]:.1f} %'
        )
        runs.append((label, stamps, intervals_ms(stamps)))

    import matplotlib  # here, not at the top: the unit tests import stats.py without matplotlib

    matplotlib.use('Agg')  # write files, open no window
    import matplotlib.pyplot as plt

    late_ms = 1.5 * 1000.0 / args.requested
    os.makedirs(args.out, exist_ok=True)

    # Histogram: how often each period happened, all runs on one plot, 1 ms bins
    lo = int(min(min(iv) for _, _, iv in runs))
    hi = int(max(max(iv) for _, _, iv in runs)) + 2
    fig, ax = plt.subplots(figsize=(8, 4.5))
    for label, _, iv in runs:
        ax.hist(iv, bins=range(lo, hi), histtype='step', linewidth=1.5, label=label)
    ax.axvline(late_ms, color='grey', linestyle='--', label=f'late: > {late_ms:.0f} ms')
    ax.set_yscale('log')  # the tail (a few long periods) is invisible on a linear axis
    ax.set_xlabel('period between /joint_states (ms)')
    ax.set_ylabel('count (log)')
    ax.legend()
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'histogram.png'), dpi=120)

    # Time series: each period over the run, one panel per run, same y axis
    fig, axes = plt.subplots(
        len(runs), 1, figsize=(10, 2.5 * len(runs)), sharey=True, squeeze=False
    )
    for ax, (label, stamps, iv) in zip(axes[:, 0], runs):
        t = [s - stamps[0] for s in stamps[1:]]  # seconds from the start of the run
        ax.plot(t, iv, '.', markersize=2)
        ax.axhline(late_ms, color='grey', linestyle='--')
        ax.set_title(label)
        ax.set_ylabel('period (ms)')
    axes[-1, 0].set_xlabel('time from the start of the run (s)')
    fig.tight_layout()
    fig.savefig(os.path.join(args.out, 'timeseries.png'), dpi=120)
    print(f'wrote histogram.png and timeseries.png to {args.out}')


if __name__ == '__main__':
    main()
