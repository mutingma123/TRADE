"""Continuous following gaps and full-body cell exclusion checks."""

import math
import numpy as np
from collections import defaultdict
from Decomposed.geometry.following import following_pairs
from ..polynomial.bernstein import segment_coefficients
from ..realization.following_intervals import following_intervals
from ..polynomial.extrema import polynomial_range

def check_pairs(scenario, segments, schedules, physics, intervals, tolerance, failures):
    minimum_gap_margin = math.inf
    gap_count = 0
    for leader, follower in following_pairs(scenario.vehicles):
        if leader.vid not in segments or follower.vid not in segments:
            continue
        for lo, hi, gap, lead, follow in following_intervals(leader, follower, segments, schedules, physics):
            difference = (np.asarray(segment_coefficients(lead, lo, hi))
                          - np.asarray(segment_coefficients(follow, lo, hi)) - gap)
            minimum, _, location, _ = polynomial_range(difference)
            minimum_gap_margin = min(minimum_gap_margin, minimum)
            gap_count += 1
            if minimum < -tolerance:
                failures.append(f"pair {leader.vid}->{follower.vid}: gap shortfall {-minimum} ft at {lo + location * (hi - lo)} s")
    by_cell = defaultdict(list)
    minimum_cell_separation = math.inf
    for interval in intervals:
        by_cell[interval["cell"]].append(interval)
        if interval["exit_s"] <= interval["entry_s"]:
            failures.append(f"vehicle {interval['vid']}, cell {interval['cell']}: nonpositive occupancy")
    for cell, rows in by_cell.items():
        rows.sort(key=lambda row: row["entry_s"])
        for index, first in enumerate(rows):
            for second in rows[index + 1:]:
                margin = second["entry_s"] - first["exit_s"]
                minimum_cell_separation = min(minimum_cell_separation, margin)
                if margin < -tolerance:
                    failures.append(f"cell {cell}: vehicles {first['vid']} and {second['vid']} overlap by {-margin} s")
    return minimum_gap_margin, gap_count, minimum_cell_separation
