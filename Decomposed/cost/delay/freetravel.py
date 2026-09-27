"""The time-optimal isolated trip over the same route and clearance target.

The benchmark uses the identical initial state, route, physical limits, and
rear clearance span as the evaluated trajectory, so the difference is delay and
not a change of definition. A turning vehicle may accelerate, cruise, and brake
before the curve, respects the turning cap only inside the conflict zone, and
re-accelerates over the downstream span.
"""

from __future__ import annotations

import math


def _accelerate_time(distance: float, initial_speed: float, cap: float, accel: float) -> float:
    """Minimum time over a distance while accelerating toward a speed cap."""
    if distance <= 0:
        return 0.0
    to_cap = max(0.0, (cap - initial_speed) / accel)
    covered = initial_speed * to_cap + 0.5 * accel * to_cap * to_cap
    if distance <= covered and to_cap > 0:
        return (-initial_speed + math.sqrt(initial_speed ** 2 + 2 * accel * distance)) / accel
    return to_cap + max(0.0, distance - covered) / cap


def _time_before_curve(runway: float, initial_speed: float, physics) -> tuple:
    """Time to the boundary and the speed there, with a braking arc if needed."""
    turn, through = physics.max_speed_turn_fps, physics.max_speed_through_fps
    accel, decel = physics.max_acc_fps2, abs(physics.max_dec_fps2)
    if initial_speed <= turn:
        return _accelerate_time(runway, initial_speed, turn, accel), min(
            turn, math.sqrt(initial_speed ** 2 + 2 * accel * runway))
    to_cap = max(0.0, (through - initial_speed) / accel)
    up = initial_speed * to_cap + 0.5 * accel * to_cap * to_cap
    down_time = (through - turn) / decel
    down = (through + turn) * down_time / 2.0
    if up + down <= runway:
        return to_cap + (runway - up - down) / through + down_time, turn
    numerator = 2 * accel * decel * runway + decel * initial_speed ** 2 + accel * turn ** 2
    peak = min(through, max(initial_speed, turn, math.sqrt(max(turn ** 2, numerator / (accel + decel)))))
    return max(0.0, (peak - initial_speed) / accel) + (peak - turn) / decel, turn


def free_travel_time(vehicle, geometry, physics) -> float:
    """Minimum travel seconds from the release state to complete clearance."""
    target = geometry.clearance_target_ft(vehicle.app, vehicle.lane, vehicle.move)
    if not vehicle.is_turning:
        return _accelerate_time(max(0.0, target - vehicle.ini_dis_ft), vehicle.ini_speed_fps,
                                physics.max_speed_through_fps, physics.max_acc_fps2)
    runway = max(0.0, physics.border_ft - vehicle.ini_dis_ft)
    inside = max(0.0, vehicle.outborder_ft - physics.border_ft)
    before, border_speed = _time_before_curve(runway, vehicle.ini_speed_fps, physics)
    turn = physics.max_speed_turn_fps
    through = _accelerate_time(inside, border_speed, turn, physics.max_acc_fps2) \
        if border_speed < turn - 1e-9 else inside / turn
    after = _accelerate_time(max(0.0, target - vehicle.outborder_ft), turn,
                             physics.max_speed_through_fps, physics.max_acc_fps2)
    return before + through + after
