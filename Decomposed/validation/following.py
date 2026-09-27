"""Continuous bumper gaps on comparable shared pavement.

A follower must stay at least one vehicle length behind its leader while both
are on pavement Geometry declares comparable. Positions are interpolated onto
a common clock, so the check does not depend on the two passes being sampled
at the same instants.
"""

from __future__ import annotations

from ..geometry.following import following_pairs, shared_segment
from ..result.trajectory.interpolation import state_at

TOL = 1e-4


def check_following(trajectory, scenario, physics) -> list:
    """Report every pair whose bumper gap goes negative while comparable."""
    failures = []
    for leader, follower in following_pairs(list(scenario.vehicles)):
        lead_rows = trajectory.points(leader.vid)
        follow_rows = trajectory.points(follower.vid)
        if not lead_rows or not follow_rows:
            continue
        segment = shared_segment(leader.move, follower.move, physics.border_ft)
        start = max(lead_rows[0].time_s, follow_rows[0].time_s)
        end = min(lead_rows[-1].time_s, follow_rows[-1].time_s)
        if end <= start:
            continue
        if _violates(lead_rows, follow_rows, segment, start, end, physics):
            failures.append(f"following gap for pair {leader.vid}->{follower.vid}")
    return failures


def _violates(lead_rows, follow_rows, segment, start, end, physics) -> bool:
    steps = int((end - start) / physics.dt_s) + 1
    for index in range(steps):
        time_s = start + index * physics.dt_s
        ahead = state_at(lead_rows, time_s)
        behind = state_at(follow_rows, time_s)
        if ahead is None or behind is None:
            continue
        if not (segment.contains(ahead[0]) and segment.contains(behind[0])):
            continue
        if ahead[0] - behind[0] - physics.vehicle_length_ft < -TOL:
            return True
    return False
