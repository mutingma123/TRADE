"""The earliest conflict-free queued pass for a signal or an all-way stop.

A signal vehicle first tries its free pass. Otherwise, and always under an
all-way stop, the vehicle stops at the first queue position whose approach
keeps the lane spacing: the line, or one slot further back per queued
vehicle. It holds there until its leader has moved on. Then, at a signal, it
departs when its cells are free and the green has begun; under an all-way
stop, it moves up, dwells at the line, and waits there until its cells are
free. Conflicts are checked cell by cell, so non-crossing paths share the box.
"""

from __future__ import annotations

import math

from ...result.trajectory.interpolation import crossing_time
from ..profiles import build_profile
from ..queueing import line_ft, queued_profile
from .reservation import conflict_push, lane_safe, occupancy_of


def queue_candidate(vehicle, accepted, occupied, geometry, physics, benchmark, *,
                    stop_at_line, earliest_entry=0.0, entry_deadline=math.inf):
    """(states, occupancy rows, entry time), or None when entry would miss the deadline."""
    guard = benchmark.reservation_guard_s

    def admits(states):
        """Entry time and the delay still needed: None past the deadline, 0 when admitted."""
        entry = crossing_time(states, physics.border_ft)
        if entry > entry_deadline:
            return entry, None
        push = conflict_push(vehicle, states, occupied, geometry, guard, False,
                             earliest_entry, physics)
        if push > 1e-7:
            return entry, push
        return entry, 0.0 if lane_safe(states, accepted, vehicle, physics) else physics.dt_s

    if not stop_at_line:
        free = build_profile(vehicle, geometry, physics, benchmark)
        entry, push = admits(free)
        if push is None:
            return None
        if push == 0.0:
            return free, occupancy_of(vehicle, free, geometry), entry
    slot = physics.vehicle_length_ft + benchmark.queue_gap_ft
    stop_ft = line_ft(physics)
    while not _approach_safe(vehicle, accepted, geometry, physics, benchmark, stop_ft,
                             stop_at_line):
        stop_ft -= slot
    hold = 0.0
    if stop_at_line:
        while True:  # hold in the slot until the lane lets the vehicle stand at the line
            states, ready = queued_profile(vehicle, geometry, physics, benchmark, stop_ft,
                                           hold, 0.0, True)
            if lane_safe(states[:ready], accepted, vehicle, physics):
                break
            hold = _longer(hold, 0.0, physics, benchmark)
    wait = 0.0
    while True:
        states, _ready = queued_profile(vehicle, geometry, physics, benchmark, stop_ft,
                                        hold if stop_at_line else wait, wait, stop_at_line)
        entry, push = admits(states)
        if push is None:
            return None
        if push == 0.0:
            return states, occupancy_of(vehicle, states, geometry), entry
        wait = _longer(wait, push, physics, benchmark)


def _approach_safe(vehicle, accepted, geometry, physics, benchmark, stop_ft, stop_at_line):
    """Whether braking to rest at `stop_ft` keeps the spacing to every queued leader."""
    states, _ready = queued_profile(vehicle, geometry, physics, benchmark, stop_ft, 0.0,
                                    stop_at_line=stop_at_line)
    rest = next(i for i, state in enumerate(states) if state.speed_fps < 1e-6)
    return lane_safe(states[:rest + 1], accepted, vehicle, physics)


def _longer(seconds, push, physics, benchmark):
    seconds += max(physics.dt_s, math.ceil(push / physics.dt_s) * physics.dt_s)
    if seconds > benchmark.max_reservation_wait_s:
        raise ValueError("No feasible queued pass within the declared wait")
    return seconds
