"""Actuated signal control: two phases, permissive lefts, and gap-out timing.

Opposing approaches share each green, and left turns yield to opposing
traffic through the cell reservations. The green rests while no conflicting
call is waiting. Once a vehicle on the other phase calls, the green ends at
gap-out after the minimum green, or at max-out counted from that call. A
vehicle that arrives during the green is detected and holds it until it
enters (presence detection), still bounded by max-out, so a yielding left is
not cut off. The next green starts after the intersection clears and the
intergreen elapses. Vehicles facing red queue at the stop line.
"""

from __future__ import annotations

import math

from ..interface import ControllerProvenance
from .dispatch import arrival_key, lane_heads, simulated_result
from .phases import phase_of, signal_phases
from .queue import queue_candidate

PROVENANCE = ControllerProvenance(
    controller="actuated",
    algorithm="Two-phase fully actuated control: permissive lefts, gap-out, max-out from "
              "the conflicting call, rest in green",
    source="Standard actuated control with declared timing parameters",
    adaptation="Phase entry authorized against the same cell occupancy and clearance rules; "
               "vehicles facing red queue at the stop line",
    parameter_version="benchmark-v2")


def green_end(benchmark, start: float, last_entry: float, call_s: float) -> tuple:
    """(gap-out end, max-out end) of the current green; unbounded while it rests."""
    max_out = max(start, call_s) + benchmark.max_green_s
    gap_out = max(start + benchmark.min_green_s, last_entry + benchmark.gap_out_s, call_s)
    return min(gap_out, max_out), max_out


def actuated(scenario, geometry, physics, benchmark):
    """Serve the phase holding the green; switch phases on a conflicting call."""
    phases, pending = signal_phases(geometry), list(scenario.vehicles)
    accepted, occupied, decisions = [], [], []
    first = min(pending, key=lambda v: arrival_key(v, physics))
    phase, start = phase_of(phases, first.app), first.release_s
    last = start
    decisions.append(dict(event="phase", apps=list(phase), time_s=start))
    while pending:
        if start > physics.horizon_s + benchmark.max_reservation_wait_s:
            raise ValueError("Actuated phases exceeded the declared reservation wait")
        heads = sorted(lane_heads(pending), key=lambda v: arrival_key(v, physics))
        waiting = [v for v in heads if v.app not in phase]
        end, max_out = green_end(benchmark, start, last,
                                 arrival_key(waiting[0], physics)[0] if waiting else math.inf)
        options, held = [], False
        for vehicle in (v for v in heads if v.app in phase):
            present = arrival_key(vehicle, physics)[0] <= end
            held = held or present
            candidate = queue_candidate(vehicle, accepted, occupied, geometry, physics, benchmark,
                                        stop_at_line=False, earliest_entry=start,
                                        entry_deadline=max_out if present else end)
            if candidate is not None:
                options.append((vehicle, candidate))
        if not options:
            cleared = max((row["exit_s"] for row in occupied), default=start)
            phase = phase_of(phases, waiting[0].app)
            start = last = max(max_out if held else end, cleared) + benchmark.intergreen_s
            decisions.append(dict(event="phase", apps=list(phase), time_s=start))
            continue
        chosen, (states, rows, entry) = min(
            options, key=lambda option: (option[1][2], arrival_key(option[0], physics)))
        last = max(last, entry)
        accepted.append((chosen, states))
        occupied.extend(rows)
        decisions.append(dict(event="reservation", vid=chosen.vid, entry_s=entry,
                              exit_s=max(row["exit_s"] for row in rows),
                              phase_apps=list(phase), controller="actuated"))
        pending.remove(chosen)
    result = simulated_result("actuated", accepted, decisions)
    result.provenance = PROVENANCE
    return result
