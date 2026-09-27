"""Continuous witness gap and monotone waiting searches."""

import math
from .mode_schedule import materialize
from ..realization.following_intervals import following_intervals
from ..polynomial.bernstein import segment_coefficients
from ..polynomial.extrema import polynomial_range

TIME_TOL_S = 1e-3

def witness(mode, wait):
    return materialize(mode, mode['entry_s'] + wait)


def gap_margin(leader, follower, lead, follow, physics):
    """Smallest excess over the required Stage 2 gap between two witnesses."""
    segments = {leader.vid: lead['witness_segments'], follower.vid: follow['witness_segments']}
    schedules = {leader.vid: lead, follower.vid: follow}
    worst = math.inf
    for lo, hi, gap, ahead, behind in following_intervals(leader, follower, segments, schedules, physics):
        difference = [a - b - gap for a, b in zip(segment_coefficients(ahead, lo, hi),
                                                   segment_coefficients(behind, lo, hi))]
        worst = min(worst, polynomial_range(difference)[0])
    return worst


def _smallest(ok, upper):
    """Smallest w in [0, upper] with ok(w), for ok monotone increasing in w."""
    if ok(0.0):
        return 0.0
    if not ok(upper):
        return None
    lo, hi = 0.0, upper
    while hi - lo > TIME_TOL_S:
        mid = (lo + hi) / 2.0
        lo, hi = (lo, mid) if ok(mid) else (mid, hi)
    return hi


def _largest(ok, upper):
    """Largest w in [0, upper] with ok(w), for ok monotone decreasing in w."""
    if not ok(0.0):
        return None
    if ok(upper):
        return upper
    lo, hi = 0.0, upper
    while hi - lo > TIME_TOL_S:
        mid = (lo + hi) / 2.0
        lo, hi = (mid, hi) if ok(mid) else (lo, mid)
    return lo
