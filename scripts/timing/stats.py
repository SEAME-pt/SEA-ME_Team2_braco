"""Statistics shared by the timing scripts (issue #90). No ROS, no serial: unit tested."""

import math


def percentile(sorted_values, p):
    """Nearest-rank percentile of an ascending list, p in 0..1 (p = 1: the max)."""
    if not sorted_values:
        raise ValueError('percentile of an empty list')
    return sorted_values[max(0, math.ceil(p * len(sorted_values)) - 1)]
