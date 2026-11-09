"""Statistics shared by the timing scripts (issue #90). No ROS, no serial: unit tested."""

import math


def percentile(sorted_values, p):
    """Nearest-rank percentile of an ascending list, p in 0..1 (p = 1: the max)."""
    if not sorted_values:
        raise ValueError('percentile of an empty list')
    return sorted_values[max(0, math.ceil(p * len(sorted_values)) - 1)]


def interval_stats(stamps_s, requested_hz):
    """Statistics of the intervals between consecutive stamps (seconds), in ms and Hz.

    late_pct: share of intervals longer than 1.5 periods of requested_hz.
    """
    if len(stamps_s) < 2:
        raise ValueError('need at least two stamps')
    iv = sorted((b - a) * 1000.0 for a, b in zip(stamps_s, stamps_s[1:]))
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
