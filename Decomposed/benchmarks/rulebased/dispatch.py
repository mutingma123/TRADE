"""The shared serve-one-lane-head-at-a-time loop.

Requests are ordered by unimpeded arrival at the intersection boundary, with
the stable vehicle identifier breaking ties, and a follower never becomes a
lane head before its leader has been served. Each rule supplies whether the
whole intersection is reserved and its guard time.
"""

from __future__ import annotations

from ..interface import ControllerResult
from .reservation import reservation_candidate


def arrival_key(vehicle, physics) -> tuple:
    """Unimpeded boundary arrival, then the stable identifier."""
    runway = physics.border_ft - vehicle.ini_dis_ft
    speed = max(vehicle.ini_speed_fps, 1e-6)
    return (vehicle.release_s + runway / speed, vehicle.vid)


def lane_heads(pending) -> list:
    """Vehicles with no unserved leader in their own approach lane."""
    return [v for v in pending
            if not any(o.app == v.app and o.lane == v.lane and o.order < v.order
                       for o in pending)]


def serve_in_order(name, scenario, geometry, physics, benchmark, *,
                   whole_box: bool, guard_s=None) -> ControllerResult:
    """Serve one lane head at a time in request order."""
    pending = list(scenario.vehicles)
    accepted, occupied, decisions = [], [], []
    while pending:
        heads = sorted(lane_heads(pending), key=lambda v: arrival_key(v, physics))
        chosen = heads[0]
        candidate = reservation_candidate(chosen, accepted, occupied, geometry, physics,
                                          benchmark, whole_box=whole_box, guard_s=guard_s)
        states, rows, entry = candidate
        accepted.append((chosen, states))
        occupied.extend(rows)
        decisions.append(dict(event="reservation", vid=chosen.vid, entry_s=entry,
                              exit_s=max(row["exit_s"] for row in rows),
                              request_s=arrival_key(chosen, physics)[0], controller=name))
        pending.remove(chosen)
    return simulated_result(name, accepted, decisions)


def simulated_result(name, accepted, decisions) -> ControllerResult:
    states = [dict(vid=state.vid, time_s=state.time_s, position_ft=state.position_ft,
                   speed_fps=state.speed_fps, acc_fps2=state.acc_fps2)
              for _vehicle, rows in accepted for state in rows]
    return ControllerResult(controller=name, status="SIMULATED", states=states,
                            decisions=decisions)
