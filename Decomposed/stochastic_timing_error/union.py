"""The strict union of open arrival windows.

Windows that overlap are merged into one interval. Windows that only touch at
an endpoint stay separate, because that endpoint is excluded from both and
merging them would add a shift value at which no overlap occurs. The result is
sorted and disjoint, which is what the Gaussian mass sum requires.
"""

from __future__ import annotations


def merge_windows(windows) -> tuple:
    """Merge overlapping open intervals; preserve excluded touching endpoints."""
    ordered = sorted(((lower, upper) for lower, upper, *_ in windows if upper > lower))
    if not ordered:
        return ()
    merged = [list(ordered[0])]
    for lower, upper in ordered[1:]:
        if lower < merged[-1][1]:
            merged[-1][1] = max(merged[-1][1], upper)
        else:
            merged.append([lower, upper])
    return tuple((lower, upper) for lower, upper in merged)


def contains(union, value: float) -> bool:
    """Strict membership in the merged union."""
    return any(lower < value < upper for lower, upper in union)
