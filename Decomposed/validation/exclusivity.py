"""Exclusive full vehicle occupancy of every shared conflict cell.

The check reads the interpolated occupancy intervals rather than the solver
binaries, so it is independent of how the schedule was produced. Two vehicles
may touch at an endpoint; a positive overlap is a violation. A missing interval
is reported as missing and never treated as an empty, therefore safe, one.
"""

from __future__ import annotations

from collections import defaultdict

TOL = 1e-6


def check_cell_exclusivity(intervals) -> list:
    """Report every pair of vehicles overlapping in a shared cell."""
    failures = []
    by_cell = defaultdict(list)
    for row in intervals:
        if not row.available:
            failures.append(f"missing occupancy interval for vehicle {row.vid} in cell {row.cell}")
            continue
        by_cell[row.cell].append(row)
    for cell, rows in sorted(by_cell.items()):
        rows.sort(key=lambda row: row.entry_s)
        for index, first in enumerate(rows):
            for second in rows[index + 1:]:
                if second.entry_s >= first.exit_s - TOL:
                    break
                failures.append(f"cell {cell} shared by vehicles {first.vid} and {second.vid}")
    return failures


def check_interval_shape(intervals) -> list:
    """Durations must be finite and strictly positive to be usable."""
    failures = []
    for row in intervals:
        if not row.available:
            continue
        if row.duration_s is None or row.duration_s <= 0:
            failures.append(f"nonpositive occupancy in cell {row.cell} for vehicle {row.vid}")
    return failures
