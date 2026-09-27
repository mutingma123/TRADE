"""Pair selection and labeling: which pairs can cross, and in which order.

A pair is eligible when the two vehicles come from different approaches and
their routes share at least one cell. Labels are fixed once, at a reference
cell chosen deterministically, so the signed entry gap at that cell is
nonnegative and every other cell's gap is measured against the same labeling.
"""

from __future__ import annotations

from collections import defaultdict
from dataclasses import dataclass


@dataclass(frozen=True)
class ConflictPair:
    first: int
    second: int
    reference_cell: int
    gamma_s: float
    shared_cells: tuple
    intervals: dict


def _by_vehicle(intervals) -> dict:
    rows = defaultdict(dict)
    for row in intervals:
        if row.available:
            rows[row.vid][row.cell] = row
    return rows


def conflicting_pairs(intervals, scenario) -> tuple:
    """Every unordered eligible pair, labeled at its reference cell."""
    rows = _by_vehicle(intervals)
    pairs = []
    ids = sorted(rows)
    for index, first in enumerate(ids):
        for second in ids[index + 1:]:
            if scenario.by_id(first).app == scenario.by_id(second).app:
                continue
            shared = tuple(sorted(set(rows[first]) & set(rows[second])))
            if not shared:
                continue
            pairs.append(_label(first, second, shared, rows))
    return tuple(pairs)


def _label(first: int, second: int, shared: tuple, rows: dict) -> ConflictPair:
    """Order the labels so the reference cell's entry gap is nonnegative."""
    reference = shared[0]
    gamma = rows[second][reference].entry_s - rows[first][reference].entry_s
    if gamma < 0:
        first, second = second, first
        gamma = -gamma
    return ConflictPair(first=first, second=second, reference_cell=reference,
                        gamma_s=gamma, shared_cells=shared,
                        intervals={cell: (rows[first][cell], rows[second][cell])
                                   for cell in shared})


def entry_gap(pair: ConflictPair, cell: int) -> float:
    """The signed local entry gap at one shared cell, under the fixed labels."""
    first, second = pair.intervals[cell]
    return second.entry_s - first.entry_s
