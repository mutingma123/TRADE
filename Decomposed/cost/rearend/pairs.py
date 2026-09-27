"""Following pairs and the pavement on which they are comparable.

A pair is surveyed only while both vehicles sit inside the shared segment
Geometry declares, so the comparison stops when the routes diverge. Unsupported
downstream merge geometry is recorded as a coverage limitation rather than
evaluated as if the two coordinates were the same pavement.
"""

from __future__ import annotations

from dataclasses import dataclass

from ...geometry.following import following_pairs, merge_support, shared_segment


@dataclass(frozen=True)
class RearPair:
    leader: int
    follower: int
    segment: object
    merge_supported: bool
    merge_note: str


def rear_pairs(scenario, physics) -> tuple:
    """Every adjacent leader and follower pair with its comparable segment."""
    pairs = []
    for leader, follower in following_pairs(list(scenario.vehicles)):
        supported, note = merge_support(leader.move, follower.move)
        pairs.append(RearPair(leader=leader.vid, follower=follower.vid,
                              segment=shared_segment(leader.move, follower.move, physics.border_ft),
                              merge_supported=supported, merge_note=note))
    return tuple(pairs)


def coverage_limits(pairs) -> tuple:
    """Pairs whose downstream geometry this cell partition does not support."""
    return tuple((pair.leader, pair.follower, pair.merge_note)
                 for pair in pairs if not pair.merge_supported)
