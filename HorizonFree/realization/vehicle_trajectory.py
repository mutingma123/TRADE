"""Build one approach and append its fixed scheduled passage."""

import math
from ..polynomial.bernstein import derivative_coefficients
from .approach_knots import approach_knots
from .approach_constraints import add_approach_constraints, DEGREE

def add_vehicle_trajectory(model, vehicle, geometry, physics, routes, schedules, n_segments, segments, objective, jerk_limit, fixed_passage_jerk_bound):
    vid = vehicle.vid
    release = float(vehicle.release_s)
    schedule = schedules[vid]
    entry, end = (float(schedule[key]) for key in ("entry_s", "exit_s"))
    passage = schedule.get("passage_segments")
    speed = float(schedule.get("entry_speed_fps", schedule.get("passage_speed_fps", 0.0)))
    entry_acc = float(schedule.get("entry_acc_fps2", 0.0))
    boundary = routes[vid][0].front_entry_ft
    target = geometry.clearance_target_ft(vehicle.app, vehicle.lane, vehicle.move)
    if (not all(math.isfinite(x) for x in (entry, end, speed, entry_acc))
            or speed < 0 or (speed == 0 and not passage)):
        raise ValueError(f"Invalid passage schedule for vehicle {vid}")
    if entry <= release or end <= entry:
        raise ValueError(f"Vehicle {vid} needs positive approach and passage durations")
    if not passage and abs(end - entry - (target - boundary) / speed) > 1e-5:
        raise ValueError(f"Vehicle {vid} clearance is inconsistent with passage speed")
    cap = physics.max_speed_turn_fps if vehicle.is_turning else physics.max_speed_through_fps
    if speed > cap + 1e-7 and (not vehicle.is_turning or entry >= release + 0.5):
        raise ValueError(f"Vehicle {vid} passage exceeds the speed cap")
    knots, cap_time = approach_knots(vehicle, schedule, release, entry, n_segments)
    segments[vid] = []
    add_approach_constraints(model, vehicle, physics, knots, cap, cap_time, boundary, speed, entry_acc, segments, objective, jerk_limit)
    if passage:
        if abs(passage[0]["start_s"] - entry) > 1e-7 or abs(passage[-1]["end_s"] - end) > 1e-7:
            raise ValueError(f"Vehicle {vid} fixed passage does not span entry to completion")
        segments[vid].extend(dict(start_s=float(seg["start_s"]), end_s=float(seg["end_s"]),
                                   coefficients_ft=list(map(float, seg["coefficients_ft"])))
                             for seg in passage)
        for seg in passage:
            duration = seg["end_s"] - seg["start_s"]
            jerk_coeff = derivative_coefficients(seg["coefficients_ft"], duration, 3)
            fixed_passage_jerk_bound += duration * sum(abs(value) for value in jerk_coeff) / len(jerk_coeff)
    else:
        segments[vid].append(dict(start_s=entry, end_s=end,
                                  coefficients_ft=[boundary + speed * (end - entry) * k / DEGREE
                                                   for k in range(DEGREE + 1)]))
    return fixed_passage_jerk_bound
