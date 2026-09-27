"""Physical checks on one trajectory: dynamics, bounds, and turning caps.

The discrete dynamics must reproduce each next state from the previous one,
the acceleration must stay inside its box and move by no more than the
declared bound per step, and the speed must respect the through cap
everywhere and the turning cap only on the relevant route segment.
"""

from __future__ import annotations

TOL = 2e-4


def check_dynamics(trajectory, scenario, physics) -> list:
    failures = []
    dt = physics.dt_s
    for vehicle in scenario.vehicles:
        rows = trajectory.points(vehicle.vid)
        for before, after in zip(rows, rows[1:]):
            predicted_speed = before.speed_fps + before.acc_fps2 * dt
            predicted_position = (before.position_ft + before.speed_fps * dt
                                  + 0.5 * before.acc_fps2 * dt * dt)
            if abs(after.speed_fps - predicted_speed) > TOL:
                failures.append(f"speed dynamics for vehicle {vehicle.vid}")
                break
            if abs(after.position_ft - predicted_position) > TOL:
                failures.append(f"position dynamics for vehicle {vehicle.vid}")
                break
            if abs(after.acc_fps2 - before.acc_fps2) > physics.max_acc_change_fps2 + 1e-4:
                failures.append(f"acceleration change for vehicle {vehicle.vid}")
                break
    return failures


def check_bounds(trajectory, scenario, physics) -> list:
    """Speed and acceleration stay inside their declared boxes."""
    failures = []
    for vehicle in scenario.vehicles:
        for state in trajectory.points(vehicle.vid):
            if state.speed_fps < -1e-5 or state.speed_fps > physics.max_speed_through_fps + 1e-4:
                failures.append(f"speed bounds for vehicle {vehicle.vid}")
                break
            if not (physics.max_dec_fps2 - 1e-5 <= state.acc_fps2 <= physics.max_acc_fps2 + 1e-5):
                failures.append(f"acceleration bounds for vehicle {vehicle.vid}")
                break
    return failures


def check_turning_caps(trajectory, scenario, physics) -> list:
    """The curve cap applies only between the boundary and the movement exit."""
    failures = []
    for vehicle in scenario.vehicles:
        if not vehicle.is_turning:
            continue
        for state in trajectory.points(vehicle.vid):
            inside = physics.border_ft <= state.position_ft <= vehicle.outborder_ft
            if inside and state.speed_fps > physics.max_speed_turn_fps + 1e-4:
                failures.append(f"turning cap for vehicle {vehicle.vid}")
                break
    return failures
