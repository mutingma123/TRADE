"""Interpolation between bracketing states, with no endpoint substitution.

A crossing time exists only when two consecutive states bracket the target
distance. When they do not, the caller receives None and records a missing
crossing, because substituting the first or last sampled time would invent an
occupancy interval that the trajectory never established.
"""

from __future__ import annotations

from bisect import bisect_left
from .passage_scan import nonmonotone_passage_times

TOL = 1e-6


def crossing_time(points, target_ft: float):
    """The instant the vehicle front reaches a distance, or None."""
    if not points:
        return None
    if abs(points[0].position_ft - target_ft) <= TOL:
        return points[0].time_s
    if points[0].position_ft > target_ft or points[-1].position_ft < target_ft - TOL:
        return None
    for before, after in zip(points, points[1:]):
        if before.position_ft <= target_ft <= after.position_ft and after.position_ft > before.position_ft:
            span = after.position_ft - before.position_ft
            fraction = (target_ft - before.position_ft) / span
            return before.time_s + fraction * (after.time_s - before.time_s)
    return None


def passage_times(points, targets_ft) -> list:
    """(first, last) instants the front is at each ascending target distance.

    The first instant is exactly `crossing_time`. The last differs only where
    the pass dwells at the target, where it is the instant the vehicle moves
    on. Nondecreasing passes use one sweep. Noisy passes visit each segment
    once and locate its target range by bisection.
    """
    monotone = all(after.position_ft >= before.position_ft
                   for before, after in zip(points, points[1:]))
    if not monotone:
        return nonmonotone_passage_times(points, targets_ft, TOL)
    times, start = [], 0
    for target in targets_ft:
        if not points:
            times.append((None, None))
            continue
        at_start = abs(points[0].position_ft - target) <= TOL
        if not at_start and (points[0].position_ft > target
                             or points[-1].position_ft < target - TOL):
            times.append((None, None))
            continue
        first = last = None
        for index in range(start if monotone else 0, len(points) - 1):
            before, after = points[index], points[index + 1]
            if monotone and before.position_ft > target:
                break
            if before.position_ft <= target <= after.position_ft and after.position_ft > before.position_ft:
                span = after.position_ft - before.position_ft
                value = before.time_s + (target - before.position_ft) / span * (after.time_s - before.time_s)
                if first is None:
                    first = value
                    start = index if monotone else 0
                last = value
        if at_start:
            first = points[0].time_s
            last = first if last is None else last
        times.append((first, last))
    return times


def state_at(points, time_s: float):
    """Linear interpolation of position and speed, or None outside the pass."""
    if not points or time_s < points[0].time_s - TOL or time_s > points[-1].time_s + TOL:
        return None
    times = [p.time_s for p in points]
    index = bisect_left(times, time_s)
    if index == 0:
        return (points[0].position_ft, points[0].speed_fps)
    if index >= len(points):
        return (points[-1].position_ft, points[-1].speed_fps)
    before, after = points[index - 1], points[index]
    span = after.time_s - before.time_s
    fraction = 0.0 if span <= 0 else (time_s - before.time_s) / span
    return (before.position_ft + fraction * (after.position_ft - before.position_ft),
            before.speed_fps + fraction * (after.speed_fps - before.speed_fps))
