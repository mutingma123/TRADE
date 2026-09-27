"""Arrival windows: the shift values at which one shared cell is overlapped.

For each shared cell the window is (gamma - gamma_o - d_second, gamma -
gamma_o + d_first), built from both vehicles' occupancy durations. Endpoints
are excluded, because a touching pair does not overlap. A nonpositive or
nonfinite duration is rejected rather than widened into a usable window.
"""

from __future__ import annotations

import math

from .pairs import entry_gap


def arrival_windows(pair) -> tuple:
    """One open window per shared cell, in the shift domain of the pair."""
    windows = []
    for cell in pair.shared_cells:
        first, second = pair.intervals[cell]
        first_duration, second_duration = first.duration_s, second.duration_s
        for duration in (first_duration, second_duration):
            if duration is None or not math.isfinite(duration) or duration <= 0:
                raise ValueError(f"Cell {cell} has an unusable occupancy duration "
                                 f"for pair {pair.first}-{pair.second}")
        local = entry_gap(pair, cell)
        lower = pair.gamma_s - local - second_duration
        upper = pair.gamma_s - local + first_duration
        windows.append((lower, upper, cell))
    return tuple(windows)
