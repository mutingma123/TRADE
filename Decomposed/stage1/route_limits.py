"""Route thresholds, cell users, and entry-speed limits for Stage 1 and execution."""

from __future__ import annotations

import math
from collections import defaultdict


def speed_cap(vehicle, physics):
    if vehicle.is_turning:
        return physics.max_speed_turn_fps
    return physics.max_speed_through_fps


def collect_routes(scenario, geometry):
    return {v.vid: geometry.thresholds(v.app, v.lane, v.move) for v in scenario.vehicles}


def cell_users(scenario, routes):
    users = defaultdict(list)
    for vehicle in scenario.vehicles:
        seen = set()
        for threshold in routes[vehicle.vid]:
            if threshold.cell not in seen:
                users[threshold.cell].append(vehicle)
                seen.add(threshold.cell)
    return dict(users)


def reachable_entry_speed(vehicle, physics, cap):
    gap = max(0.0, physics.border_ft - vehicle.ini_dis_ft)
    vmax = math.sqrt(max(0.0, vehicle.ini_speed_fps ** 2 + 2.0 * physics.max_acc_fps2 * gap))
    vmin_sq = vehicle.ini_speed_fps ** 2 + 2.0 * physics.max_dec_fps2 * gap
    vmin = math.sqrt(vmin_sq) if vmin_sq > 0.0 else 0.0
    return min(vmin, cap), min(vmax, cap)

