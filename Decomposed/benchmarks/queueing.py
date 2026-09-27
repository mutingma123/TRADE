"""Stop-line queueing on the shared kinematics: approach, stop, hold, and go.

A vehicle that must yield keeps its speed, then brakes, easing in and out
under the jerk bound, to come to rest at the stop line or at a queue slot
behind it. It holds there and then departs. Under an all-way stop, a vehicle
held in the queue first moves up and stops again at the line, and every
vehicle dwells at the line before it enters. Braking that starts on the first
state needs no ease-in, because no earlier acceleration constrains it.
"""

from __future__ import annotations

import math

from .profiles import cruise_cap, ramp
from .stepping import Path

LINE_TOL_FT = 3.0  # a vehicle at rest this close to the line has stopped at it


def line_ft(physics) -> float:
    return physics.border_ft - 0.5


def _travel(speed, accs, dt) -> float:
    position = 0.0
    for acc in accs:
        position += speed * dt + 0.5 * acc * dt * dt
        speed = max(0.0, speed + acc * dt)
    return position


def _fit(speed, gap, physics, ease_in):
    """The longest braking of this shape that still stops within the gap."""
    count, best = len(ramp(-speed, physics, from_rest=ease_in)), None
    while True:
        accs = ramp(-speed, physics, from_rest=ease_in, steps=count)
        if _travel(speed, accs, physics.dt_s) > gap + 1e-9:
            return best
        best, count = accs, count + 1


def stop_within(speed, gap, physics):
    """Accelerations that bring the vehicle to rest as close to `gap` ft ahead as possible."""
    if speed <= 1e-9:
        return [] if gap >= -1e-9 else None
    firm = _travel(speed, ramp(-speed, physics, from_rest=True), physics.dt_s)
    if gap < firm:
        return _fit(speed, gap, physics, ease_in=False)
    cruise = int((gap - firm) // (speed * physics.dt_s))
    return [0.0] * cruise + _fit(speed, gap - cruise * speed * physics.dt_s, physics, True)


def move_up(distance, physics) -> list:
    """A rest-to-rest move over `distance` ft that respects the jerk bound."""
    count = max(5, math.ceil(math.sqrt(distance / min(physics.max_acc_fps2, -physics.max_dec_fps2))
                             / physics.dt_s) + 6)
    while True:
        peak = distance / ((count + 1) * physics.dt_s)
        try:
            up = ramp(peak, physics, from_rest=True, steps=count)
            return up + [0.0] + [-acc for acc in reversed(up)]
        except ValueError:
            count += 1


def _hold(path, seconds, minimum=0) -> None:
    for _ in range(max(minimum, math.ceil(seconds / path.physics.dt_s - 1e-8))):
        path.step(0.0)


def queued_profile(vehicle, geometry, physics, benchmark, stop_ft, hold_s, wait_s=0.0,
                   stop_at_line=False):
    """States and the index where the vehicle is ready to go: `hold_s` is spent
    where it first stops (its slot); with a stop at the line, `wait_s` follows the dwell."""
    path = Path(vehicle, physics)
    brake = stop_within(path.clock["speed_fps"], stop_ft - path.clock["position_ft"], physics)
    if brake is None:
        raise ValueError(f"Vehicle {vehicle.vid} cannot stop by {stop_ft:.1f} ft")
    for acc in brake:
        path.step(acc)
    short = line_ft(physics) - path.clock["position_ft"]
    if not stop_at_line:
        _hold(path, hold_s, minimum=1)
        ready = len(path.states)
    else:
        if short > LINE_TOL_FT:
            _hold(path, hold_s, minimum=1)
            for acc in move_up(short, physics):
                path.step(acc)
        _hold(path, benchmark.stop_dwell_s, minimum=1)
        ready = len(path.states)
        _hold(path, wait_s)
    cap = cruise_cap(vehicle, physics)
    for acc in ramp(cap, physics, from_rest=True):
        path.step(acc)
    return path.leave(cap, geometry), ready
