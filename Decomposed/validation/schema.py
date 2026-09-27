"""Schema, identity, unit, and clock checks on a normalized trajectory.

The identities must match the scenario exactly, every value must be finite,
and the samples must sit on the declared time grid starting at each vehicle's
own release. These checks run before any physical check, because a mismatched
identity makes every later comparison meaningless.
"""

from __future__ import annotations

import math

TOL = 1e-5


def check_identities(trajectory, scenario) -> list:
    expected = {v.vid for v in scenario.vehicles}
    present = set(trajectory.vehicle_ids())
    failures = []
    if present != expected:
        failures.append(f"identity mismatch: missing {sorted(expected - present)}, "
                        f"unexpected {sorted(present - expected)}")
    for vid, rows in trajectory.states.items():
        if any(state.vid != vid for state in rows):
            failures.append(f"state identity does not match vehicle {vid}")
    if trajectory.scenario_hash != scenario.scenario_hash:
        failures.append("the trajectory was produced under a different scenario")
    return failures


def check_finite(trajectory) -> list:
    failures = []
    for vid, rows in trajectory.states.items():
        for state in rows:
            values = (state.time_s, state.position_ft, state.speed_fps, state.acc_fps2)
            if not all(math.isfinite(value) for value in values):
                failures.append(f"nonfinite state for vehicle {vid} at {state.time_s}")
                break
    return failures


def check_clock(trajectory, scenario, physics) -> list:
    """Samples sit on the grid, start at the release, and advance in time."""
    failures = []
    for vehicle in scenario.vehicles:
        rows = trajectory.points(vehicle.vid)
        if not rows:
            failures.append(f"no states for vehicle {vehicle.vid}")
            continue
        if abs(rows[0].time_s - vehicle.release_s) > TOL:
            failures.append(f"vehicle {vehicle.vid} does not start at its release")
        for before, after in zip(rows, rows[1:]):
            if abs(after.time_s - before.time_s - physics.dt_s) > TOL:
                failures.append(f"vehicle {vehicle.vid} leaves the declared time grid")
                break
            if after.position_ft < before.position_ft - TOL:
                failures.append(f"vehicle {vehicle.vid} moves backward")
                break
    return failures


def check_initial_state(trajectory, scenario) -> list:
    failures = []
    for vehicle in scenario.vehicles:
        rows = trajectory.points(vehicle.vid)
        if not rows:
            continue
        if (abs(rows[0].position_ft - vehicle.ini_dis_ft) > 1e-4
                or abs(rows[0].speed_fps - vehicle.ini_speed_fps) > 1e-4):
            failures.append(f"vehicle {vehicle.vid} does not start at its scenario state")
    return failures
