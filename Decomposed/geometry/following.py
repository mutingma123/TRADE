"""FollowingSegments: where two vehicles share comparable pavement.

Two vehicles in the same approach lane share the approach up to the
intersection boundary. They keep sharing pavement past the boundary only when
their movements are identical, because the one dimensional travel coordinate
means different things on different routes. Downstream merge geometry between
distinct routes is not supported and is recorded as missing rather than
silently compared.

The approach extends upstream of coordinate zero. Wider initial queues can
start there, so comparisons use the full recorded approach, including negative
coordinates, and never hide an early following contact upstream of zero.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class FollowingSegment:
    shared: bool
    lower_ft: float
    upper_ft: float
    reason: str

    def contains(self, position_ft: float) -> bool:
        return self.shared and self.lower_ft <= position_ft <= self.upper_ft


def shared_segment(lead_move: int, follow_move: int, border_ft: float) -> FollowingSegment:
    """The longitudinal window on which a leader and follower are comparable."""
    if lead_move == follow_move:
        return FollowingSegment(True, -math.inf, math.inf, "identical route")
    return FollowingSegment(True, -math.inf, border_ft, "shared approach only")


def merge_support(lead_move: int, follow_move: int) -> tuple:
    """Whether downstream merging of two distinct routes is modeled.

    It is not. The pair is compared on the shared approach and the divergence
    is reported, so an unsupported merge appears as a declared coverage limit
    rather than as a false safe gap.
    """
    if lead_move == follow_move:
        return True, "same movement, no merge"
    return False, "downstream merge geometry is outside this cell partition"


def following_pairs(vehicles) -> tuple:
    """Adjacent leader and follower pairs within each approach lane."""
    lanes: dict = {}
    for vehicle in vehicles:
        lanes.setdefault((vehicle.app, vehicle.lane), []).append(vehicle)
    pairs = []
    for group in lanes.values():
        group.sort(key=lambda v: v.order)
        pairs.extend(zip(group, group[1:]))
    return tuple(pairs)
