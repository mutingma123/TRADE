"""Criteria read off a realized trajectory, identical for every producer."""

from Decomposed.result.trajectory.clearance import clearance_of
from Decomposed.result.trajectory.interpolation import crossing_time
from Decomposed.result.trajectory.intervals import cell_intervals
from Decomposed.stage1.route_limits import reachable_entry_speed, speed_cap


def event_fidelity(intervals, schedules):
    """Largest distance between a realized cell event and its scheduled time."""
    deviations = []
    for row in intervals:
        schedule = schedules[row["vid"]]
        deviations.append(abs(row["entry_s"] - schedule["cell_entries_s"][row["cell"]]))
        deviations.append(abs(row["exit_s"] - schedule["cell_exits_s"][row["cell"]]))
    return dict(events=len(intervals),
                max_deviation_s=max(deviations) if deviations else None,
                mean_deviation_s=sum(deviations) / len(deviations) if deviations else None)


def realized_metrics(trajectory, scene, geometry, physics, routes, exact_variation=None):
    """Criteria read off a realized trajectory, identical for every producer."""
    clearances = {record.vid: record for record in clearance_of(trajectory, scene, geometry)}
    speeds, travel, variation = [], [], 0.0
    for vehicle in scene.vehicles:
        points = trajectory.points(vehicle.vid)
        boundary = routes[vehicle.vid][0].front_entry_ft
        entry = crossing_time(points, boundary)
        cap = reachable_entry_speed(vehicle, physics, speed_cap(vehicle, physics))[1]
        if entry is not None and entry > vehicle.release_s:
            speeds.append(min(cap, (boundary - vehicle.ini_dis_ft) / (entry - vehicle.release_s)))
        clearance = clearances[vehicle.vid].clearance_s
        if clearance is not None:
            travel.append(clearance - vehicle.release_s)
            if exact_variation is None:
                variation += sum(abs(after.acc_fps2 - before.acc_fps2)
                                 for before, after in zip(points, points[1:])
                                 if after.time_s <= clearance + 1e-9)
    complete = len(travel) == scene.n_vehicles
    intervals = [row for row in cell_intervals(trajectory, scene, geometry) if row.available]
    hinge = 0.0
    for k, first in enumerate(intervals):
        for second in intervals[k + 1:]:
            if first.cell == second.cell and first.app != second.app:
                hinge += max(0.0, physics.crossing_window_s - abs(first.entry_s - second.entry_s))
    return dict(
        complete=complete,
        mean_capped_approach_speed_fps=(sum(speeds) / len(speeds)
                                        if len(speeds) == scene.n_vehicles else None),
        realized_event_hinge_s=hinge,
        mean_travel_s=sum(travel) / len(travel) if complete else None,
        last_clearance_s=max(clearances[v.vid].clearance_s for v in scene.vehicles) if complete else None,
        acceleration_variation_fps2=(exact_variation if exact_variation is not None
                                     else variation if complete else None),
        variation_definition=("exact integral of |jerk| through each clearance"
                              if exact_variation is not None
                              else "sum of acceleration jumps between samples through each clearance"))
