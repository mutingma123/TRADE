"""The shared reservation search every practical rule uses.

A candidate schedule is proposed, its occupancy is compared against the
reservations already accepted, and the release is pushed until no conflict and
no following violation remains. The guard time and whether the whole
intersection or only the shared cells are reserved distinguish the rules; the
physics do not change between them.
"""

from __future__ import annotations

import math

from ...result.trajectory.interpolation import crossing_time, state_at
from ..profiles import build_profile

TOL = 1e-8


def occupancy_of(vehicle, states, geometry) -> tuple:
    """Cell entry and exit times for one candidate pass."""
    rows = []
    for threshold in geometry.thresholds(vehicle.app, vehicle.lane, vehicle.move):
        entry = crossing_time(states, threshold.front_entry_ft)
        exit_s = crossing_time(states, threshold.rear_exit_ft)
        if entry is None or exit_s is None:
            raise ValueError(f"Candidate pass for vehicle {vehicle.vid} misses cell {threshold.cell}")
        rows.append(dict(vid=vehicle.vid, cell=threshold.cell, entry_s=entry, exit_s=exit_s))
    return tuple(rows)


def lane_safe(states, accepted, vehicle, physics) -> bool:
    """No accepted schedule in the same lane is overtaken or crowded."""
    for other_vehicle, other_states in accepted:
        if (other_vehicle.app, other_vehicle.lane) != (vehicle.app, vehicle.lane):
            continue
        if other_vehicle.order >= vehicle.order:
            raise ValueError("A follower was scheduled before its leader")
        same_route = other_vehicle.move == vehicle.move
        for point in states:
            ahead = state_at(other_states, point.time_s)
            if ahead is None:
                continue
            if not same_route and (ahead[0] >= physics.border_ft or point.position_ft >= physics.border_ft):
                continue
            gap = (physics.vehicle_length_ft + physics.safe_buffer_ft if same_route and point.position_ft >= physics.border_ft
                   else physics.vehicle_length_ft)
            if ahead[0] - point.position_ft < gap - 1e-5:
                return False
    return True


def conflict_push(vehicle, states, occupied, geometry, guard_s, whole_box, earliest_entry, physics) -> float:
    """How far this candidate must be delayed to clear every conflict."""
    entry = crossing_time(states, physics.border_ft)
    push = max(0.0, earliest_entry - entry)
    rows = occupancy_of(vehicle, states, geometry)
    if whole_box:
        exit_s = max(row["exit_s"] for row in rows)
        for busy in occupied:
            if min(exit_s, busy["exit_s"]) + guard_s > max(entry, busy["entry_s"]) + TOL:
                push = max(push, busy["exit_s"] + guard_s - entry)
    else:
        for row in rows:
            for busy in occupied:
                if row["cell"] != busy["cell"]:
                    continue
                if min(row["exit_s"], busy["exit_s"]) + guard_s > max(row["entry_s"], busy["entry_s"]) + TOL:
                    push = max(push, busy["exit_s"] + guard_s - row["entry_s"])
    return push


def reservation_candidate(vehicle, accepted, occupied, geometry, physics, benchmark, *,
                          whole_box=False, guard_s=None,
                          earliest_entry=0.0, entry_deadline=math.inf, cruise_speed=None):
    """The earliest conflict-free pass for one vehicle, or None before a deadline."""
    guard_s = benchmark.reservation_guard_s if guard_s is None else guard_s
    wait = None
    while True:
        states = build_profile(vehicle, geometry, physics, benchmark, wait_s=wait,
                               cruise_speed=cruise_speed)
        entry = crossing_time(states, physics.border_ft)
        if entry is None:
            raise ValueError(f"Candidate pass for vehicle {vehicle.vid} never reaches the boundary")
        if entry > entry_deadline:
            return None
        push = conflict_push(vehicle, states, occupied, geometry, guard_s, whole_box,
                             earliest_entry, physics)
        if push <= 1e-7 and lane_safe(states, accepted, vehicle, physics):
            return states, occupancy_of(vehicle, states, geometry), entry
        step = max(physics.dt_s, math.ceil(push / physics.dt_s) * physics.dt_s)
        wait = 0.0 if wait is None else wait + step
        if wait > benchmark.max_reservation_wait_s:
            raise ValueError(f"No feasible reservation for vehicle {vehicle.vid} within the declared wait")
