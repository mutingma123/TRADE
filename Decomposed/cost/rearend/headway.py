"""Following margins: how much earlier a follower must run to reach its leader.

At lane position p the follower's front arrives at T_f(p), and the leader's rear
clears p when the leader's front reaches p plus one vehicle length. The margin
T_f(p) - T_l(p + length) is the arrival error difference at which the
follower's front meets the leader's rear there. Both passing times are
piecewise linear between recorded states and jump only where a vehicle dwells,
so the smallest margin over the shared pavement is found exactly at the
recorded positions of either pass, taking the departure instant just past a
dwell. Comparison starts where the leader's rear stood when its record begins.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from ...result.trajectory.interpolation import passage_times
from .pairs import rear_pairs


@dataclass(frozen=True)
class FollowingMargin:
    rear: object
    min_headway_s: Optional[float]
    breakpoints: int

    @property
    def compared(self) -> bool:
        """Whether the pair shared any pavement both records cover."""
        return self.min_headway_s is not None


def pair_margin(rear, lead_rows, follow_rows, vehicle_length_ft: float) -> FollowingMargin:
    """The smallest follower arrival minus leader rear clearance on shared pavement."""
    if not lead_rows or not follow_rows:
        return FollowingMargin(rear=rear, min_headway_s=None, breakpoints=0)
    start = max(rear.segment.lower_ft, follow_rows[0].position_ft,
                lead_rows[0].position_ft - vehicle_length_ft)
    stop = min(rear.segment.upper_ft, follow_rows[-1].position_ft,
               lead_rows[-1].position_ft - vehicle_length_ft)
    if stop < start:
        return FollowingMargin(rear=rear, min_headway_s=None, breakpoints=0)
    positions = sorted({start, stop}
                       | {row.position_ft for row in follow_rows if start <= row.position_ft <= stop}
                       | {row.position_ft - vehicle_length_ft for row in lead_rows
                          if start <= row.position_ft - vehicle_length_ft <= stop})
    follow = passage_times(follow_rows, positions)
    lead = passage_times(lead_rows, [position + vehicle_length_ft for position in positions])
    margins = []
    for position, (follow_first, follow_last), (lead_first, lead_last) in zip(positions, follow, lead):
        if follow_first is not None and lead_first is not None:
            margins.append(follow_first - lead_first)
        if position < stop and follow_last is not None and lead_last is not None:
            margins.append(follow_last - lead_last)
    return FollowingMargin(rear=rear, min_headway_s=min(margins) if margins else None,
                           breakpoints=len(positions))


def following_margins(trajectory, scenario, physics) -> tuple:
    """The smallest margin of every adjacent following pair on its shared pavement."""
    return tuple(pair_margin(rear, trajectory.points(rear.leader),
                             trajectory.points(rear.follower), physics.vehicle_length_ft)
                 for rear in rear_pairs(scenario, physics))
