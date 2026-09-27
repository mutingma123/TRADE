"""Piecewise constant acceleration profiles on the shared discrete kinematics.

A benchmark schedule obeys the same step size, acceleration box, acceleration
change bound, turning cap, and clearance target as an optimized one, so the two
are comparable. The profile either passes freely or brakes, waits, and departs;
stop-line queueing lives in `queueing`.
"""

from __future__ import annotations

import math

from .stepping import Path


def ramp(delta_speed: float, physics, from_rest: bool = False, steps: int = None) -> list:
    """Accelerations that change speed by delta while respecting the jerk bound."""
    if abs(delta_speed) < 1e-9:
        return []
    limit = physics.max_acc_fps2 if delta_speed > 0 else -physics.max_dec_fps2
    sign = 1.0 if delta_speed > 0 else -1.0
    count = steps or max(1, math.ceil(abs(delta_speed) / (limit * physics.dt_s)))
    while True:
        weights = [min(limit, physics.max_acc_change_fps2 * (count - i),
                       physics.max_acc_change_fps2 * (i + 1) if from_rest else limit)
                   for i in range(count)]
        capacity = sum(weights) * physics.dt_s
        if capacity >= abs(delta_speed) - 1e-8:
            scale = abs(delta_speed) / capacity
            return [sign * weight * scale for weight in weights]
        if steps is not None:
            raise ValueError("The requested profile length cannot reach that speed")
        count += 1


def cruise_cap(vehicle, physics, cruise_speed: float = None) -> float:
    if cruise_speed is not None:
        return cruise_speed
    return physics.max_speed_turn_fps if vehicle.is_turning else physics.max_speed_through_fps


def build_profile(vehicle, geometry, physics, benchmark, wait_s: float = None,
                  cruise_speed: float = None) -> list:
    """One vehicle's states: a free pass, or brake to rest at once, wait, and depart."""
    cap = cruise_cap(vehicle, physics, cruise_speed)
    path = Path(vehicle, physics)
    if wait_s is None:
        for acc in ramp(cap - path.clock["speed_fps"], physics):
            path.step(acc)
    else:
        for acc in ramp(-path.clock["speed_fps"], physics):
            path.step(acc)
        if path.clock["position_ft"] > physics.border_ft - 0.5:
            raise ValueError(f"Vehicle {vehicle.vid} cannot stop before the intersection")
        for _ in range(max(1, math.ceil(wait_s / physics.dt_s - 1e-8))):
            path.step(0.0)
        for acc in ramp(cap, physics, from_rest=True):
            path.step(acc)
    return path.leave(cap, geometry)
