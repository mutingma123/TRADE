"""All-way stop: every vehicle stops at the line; the first to stop goes first.

Each vehicle approaches, queues behind any vehicle waiting in its lane, comes
to rest at the stop line, and dwells. Right of way follows the order in which
vehicles come to rest at the line, ties broken by vehicle identifier. A
vehicle then enters as soon as none of its cells is occupied, so opposing
through movements and other non-crossing paths proceed together rather than
the whole box being held for one vehicle at a time. The dwell is recorded
with the schedule because it is a declared assumption, not a measured value.
"""

from __future__ import annotations

from ..interface import ControllerProvenance
from ..queueing import LINE_TOL_FT, line_ft
from .dispatch import lane_heads, simulated_result
from .queue import queue_candidate

PROVENANCE = ControllerProvenance(
    controller="allwaystop",
    algorithm="Mandatory stop at the line, first-stopped first-served, "
              "non-conflicting movements proceed together",
    source="Standard all-way stop right-of-way rule with a declared dwell",
    adaptation="Stop-line queueing and cell-level conflicts on the 16-cell geometry",
    parameter_version="benchmark-v2")


def allwaystop(scenario, geometry, physics, benchmark):
    """Queue, stop, and release lane heads in the order they stop at the line."""
    pending = list(scenario.vehicles)
    accepted, occupied, decisions, stopped = [], [], [], {}
    while pending:
        heads = lane_heads(pending)
        for vehicle in heads:
            if vehicle.vid not in stopped:  # fixed once its lane leader has been served
                stopped[vehicle.vid] = _stopped_s(vehicle, accepted, geometry, physics, benchmark)
        chosen = min(heads, key=lambda v: (stopped[v.vid], v.vid))
        states, rows, entry = queue_candidate(chosen, accepted, occupied, geometry, physics,
                                              benchmark, stop_at_line=True)
        accepted.append((chosen, states))
        occupied.extend(rows)
        decisions.append(dict(event="reservation", vid=chosen.vid, entry_s=entry,
                              exit_s=max(row["exit_s"] for row in rows),
                              stopped_s=stopped[chosen.vid], controller="allwaystop"))
        pending.remove(chosen)
    result = simulated_result("allwaystop", accepted, decisions)
    result.provenance = PROVENANCE
    return result


def _stopped_s(vehicle, accepted, geometry, physics, benchmark) -> float:
    """When the vehicle first stands at the line, queueing but ignoring the box."""
    states = queue_candidate(vehicle, accepted, (), geometry, physics, benchmark,
                             stop_at_line=True)[0]
    line = line_ft(physics) - LINE_TOL_FT
    return next(s.time_s for s in states if s.speed_fps < 1e-6 and s.position_ft >= line)
