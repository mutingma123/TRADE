"""Feasibility checks on a generated batch before any controller or solve sees it.

Validation rejects duplicate identities, illegal routes, vehicles starting
inside the intersection or above the speed cap, overlapping same lane
vehicles, followers released before their leaders, and initial following gaps
whose conservative contact bound exceeds the configured rear-end risk limit.
"""

from __future__ import annotations

import math


def validate_vehicles(vehicles, physics, geometry) -> None:
    """Reject duplicate identities, illegal routes, and infeasible queues."""
    if len({v.vid for v in vehicles}) != len(vehicles):
        raise ValueError("Duplicate vehicle identifiers")
    lanes: dict = {}
    for vehicle in vehicles:
        geometry.route(vehicle.app, vehicle.lane, vehicle.move)
        if vehicle.ini_speed_fps > physics.max_speed_through_fps:
            raise ValueError(f"Vehicle {vehicle.vid} starts above the speed cap")
        if vehicle.ini_dis_ft >= physics.border_ft:
            raise ValueError(f"Vehicle {vehicle.vid} starts inside the intersection")
        lanes.setdefault((vehicle.app, vehicle.lane), []).append(vehicle)
    for group in lanes.values():
        group.sort(key=lambda v: v.order)
        if [v.order for v in group] != list(range(len(group))):
            raise ValueError("Queue orders must be contiguous from zero")
        for lead, follower in zip(group, group[1:]):
            if lead.ini_dis_ft - follower.ini_dis_ft < physics.vehicle_length_ft:
                raise ValueError("Same lane vehicles overlap at their initial states")
            if follower.release_s < lead.release_s - 1e-9:
                raise ValueError("A follower is released before its leader")
            headway, probability = initial_rear_bound(lead, follower, physics)
            if probability > physics.initial_rear_probability_limit + 1e-12:
                raise ValueError("Initial following gap exceeds the configured rear-end risk limit: "
                                 f"pair {lead.vid}->{follower.vid}, bound={probability:.6g}, "
                                 f"headway={headway:.6g} s")


def initial_rear_bound(lead, follower, physics) -> tuple:
    """Conservative initial contact bound, including ordered release times."""
    gap = lead.ini_dis_ft - follower.ini_dis_ft - physics.vehicle_length_ft
    headway = follower.release_s - lead.release_s + gap / physics.max_speed_through_fps
    probability = 0.5 * math.erfc(headway / (2.0 * physics.initial_rear_sigma_s))
    return headway, probability
