"""Attainable cruise and stopping motion modes."""

from .motion_builder import Motion
from .threshold_crossings import crossing, _finish
from ..polynomial.bernstein import evaluate_segment

def build_modes(vehicle, geometry, physics, routes):
    boundary = routes[0].front_entry_ft
    target = geometry.clearance_target_ft(vehicle.app, vehicle.lane, vehicle.move)
    cap = physics.max_speed_turn_fps if vehicle.is_turning else physics.max_speed_through_fps
    speeds = [1, 3, 5, 10, 15, 20, 25, 30, 35, 39, 40, 45, 50, 55, 60]
    modes = []
    for cruise in [s for s in speeds if s <= cap] + [0]:
        motion = Motion(vehicle)
        motion.transition(cruise, physics, free_initial=True)
        if motion.x >= boundary - 1e-6:
            continue
        wait_start = motion.t if cruise == 0 else None
        wait_position = motion.x if cruise == 0 else None
        if cruise > 0:
            motion.add((boundary - motion.x) / cruise)
        motion.transition(cap, physics)
        finish = _finish(motion, target, cap)
        entry = crossing(motion.pieces, boundary)
        # Apply the stronger turning convention from 0.5 s after release.
        if vehicle.is_turning:
            at = vehicle.release_s + 0.5
            piece = next(p for p in motion.pieces if p['start_s'] - 1e-9 <= at <= p['end_s'] + 1e-9)
            if evaluate_segment(piece, at, 1) > cap + 1e-6:
                continue
        first = next(p for p in motion.pieces if p['start_s'] - 1e-9 <= entry <= p['end_s'] + 1e-9)
        entries = {th.cell: crossing(motion.pieces, th.front_entry_ft) - entry for th in routes}
        exits = {th.cell: crossing(motion.pieces, th.rear_exit_ft) - entry for th in routes}
        modes.append(dict(name='stop' if cruise == 0 else f'cruise_{cruise}',
                          entry_s=entry, duration_s=finish - entry,
                          entry_speed_fps=evaluate_segment(first, entry, 1),
                          entry_acc_fps2=evaluate_segment(first, entry, 2),
                          cap_fps=cap, wait_start_s=wait_start, wait_position_ft=wait_position,
                          pieces=motion.pieces, entries=entries, exits=exits,
                          speed_deficit_ft=cap * (finish - entry) - (target - boundary)))
    if not modes or not any(m['name'] == 'stop' for m in modes):
        raise ValueError(f'Vehicle {vehicle.vid} cannot stop before entry with these physical limits')
    return modes
