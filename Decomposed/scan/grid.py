"""Instance-relative ceilings on the crossing criterion.

Caps are relative to this instance's own anchors, because an absolute grid
would make ceilings incomparable across arrival realizations. Index zero is the
low proximity anchor and the last index is the high speed anchor, so the
normalized index supports paired traffic contrasts even though the absolute
cap values differ across scenarios.
"""

from __future__ import annotations

DEGENERATE_TOL = 1e-9


def normalized_caps(low: float, high: float, count: int) -> tuple:
    """(index, normalized position, absolute cap) for every planned index."""
    if count < 2:
        raise ValueError("A criterion scan needs both anchors")
    if high < low - DEGENERATE_TOL:
        raise ValueError(f"The high anchor cap {high} lies below the low anchor {low}")
    span = max(0.0, high - low)
    if span <= DEGENERATE_TOL:
        return tuple((k, k / (count - 1), float(high)) for k in range(count))
    return tuple((k, k / (count - 1), low + span * k / (count - 1)) for k in range(count))

