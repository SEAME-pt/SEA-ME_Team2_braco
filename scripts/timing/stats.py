"""Statistics shared by the timing scripts. No ROS, no serial: unit tested."""

import math


def percentile(sorted_values, p):
    """Nearest-rank percentile of an ascending list, p in 0..1 (p = 1: the max)."""
    if not sorted_values:
        raise ValueError('percentile of an empty list')
    return sorted_values[max(0, math.ceil(p * len(sorted_values)) - 1)]


def load_stamps(path):
    """Stamps (seconds) from a js_rate.py --raw CSV: a 'stamp_s' header, one stamp per line."""
    with open(path) as f:
        lines = f.read().split()
    if not lines or lines[0] != 'stamp_s':
        raise ValueError(f'{path}: not a js_rate.py --raw CSV (no stamp_s header)')
    return [float(x) for x in lines[1:]]


def intervals_ms(stamps_s):
    """Intervals between consecutive stamps in ms, in time order (for a time series)."""
    return [(b - a) * 1000.0 for a, b in zip(stamps_s, stamps_s[1:])]


def interval_stats(stamps_s, requested_hz):
    """Statistics of the intervals between consecutive stamps (seconds), in ms and Hz.

    late_pct: share of intervals longer than 1.5 periods of requested_hz.
    """
    if len(stamps_s) < 2:
        raise ValueError('need at least two stamps')
    iv = sorted(intervals_ms(stamps_s))
    total = sum(iv)
    if total <= 0:
        raise ValueError('the stamps do not advance')
    period_ms = 1000.0 / requested_hz
    return {
        'achieved_hz': 1000.0 * len(iv) / total,
        'mean_ms': total / len(iv),
        'p50_ms': percentile(iv, 0.50),
        'p99_ms': percentile(iv, 0.99),
        'max_ms': iv[-1],
        'late_pct': 100.0 * sum(1 for x in iv if x > 1.5 * period_ms) / len(iv),
    }


def windowed_speeds(points, window_s):
    """|velocity| over a window instead of between two samples (deg/s): one value per sample,
    from it to the first sample at least window_s later. points: (time s, angle deg), ascending.

    Between two /joint_states 20-60 ms apart a slow joint moves only a few encoder steps, so a
    one-step error looks like jitter; over ~200 ms it is small.
    """
    speeds = []
    k = 0
    for i, (ta, aa) in enumerate(points):
        k = max(k, i + 1)
        while k < len(points) and points[k][0] - ta < window_s:
            k += 1
        if k == len(points):
            break
        tb, ab = points[k]
        speeds.append(abs(ab - aa) / (tb - ta))
    return speeds


def settle_time(samples, t0, tol_deg, consecutive=3, rest_s=0.3):
    """Seconds after t0 until every joint stays within tol_deg of where it comes to rest.

    samples: (time s, [angle deg per joint]), ascending. The rest pose is the mean of the last
    rest_s seconds. Settled = `consecutive` samples in a row inside the tolerance; 0.0 if it
    already was at t0, None if it never settles (wait for this before the gripper acts).
    """
    after = [(t, a) for t, a in samples if t >= t0]
    tail = [a for t, a in after if t >= after[-1][0] - rest_s] if after else []
    if len(after) < consecutive or not tail:
        return None
    rest = [sum(col) / len(col) for col in zip(*tail)]
    run = 0
    for i, (_, a) in enumerate(after):
        run = run + 1 if all(abs(x - r) <= tol_deg for x, r in zip(a, rest)) else 0
        if run == consecutive:
            return max(0.0, after[i - consecutive + 1][0] - t0)
    return None
