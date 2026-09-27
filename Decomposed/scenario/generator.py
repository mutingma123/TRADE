"""Scenario: the immutable batch every downstream component receives.

Generation composes the independent streams: initial states, movement
allocation, and releases. The scenario hash covers the case, the seed, the
geometry identity, and the resulting vehicles, so a solve or a controller run
can prove it acted on this instance.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..config.hashing import fingerprint
from .arrivals import release_times
from .feasibility import validate_vehicles
from .movements import allocate_turns
from .states import InitialStateBounds, initial_states
from .vehicle import Vehicle

LANES = [(app, lane) for app in (1, 2, 3, 4) for lane in (1, 2)]


@dataclass(frozen=True)
class Scenario:
    case_name: str
    seed: int
    vehicles: tuple
    geometry_hash: str
    geometry_version: str

    @property
    def n_vehicles(self) -> int:
        return len(self.vehicles)

    def by_id(self, vid: int) -> Vehicle:
        for vehicle in self.vehicles:
            if vehicle.vid == vid:
                return vehicle
        raise KeyError(f"Vehicle {vid} is not in scenario {self.case_name}/{self.seed}")

    @property
    def scenario_hash(self) -> str:
        payload = dict(case=self.case_name, seed=self.seed,
                       geometry=self.geometry_hash,
                       vehicles=[v.to_dict() for v in self.vehicles])
        return fingerprint(payload)


def build_scenario(case, seed: int, physics, geometry,
                   bounds: InitialStateBounds = InitialStateBounds()) -> Scenario:
    """Generate one reproducible vehicle batch for a case and seed."""
    states = initial_states(seed, case.n_per_lane, physics, bounds)
    releases = release_times(case, seed, physics)
    vehicles = []
    for order in range(case.n_per_lane):
        moves = allocate_turns(seed, order, case.turn_fraction)
        for app, lane in LANES:
            move = moves[(app, lane)]
            position, speed = states[(app, lane, order)]
            vehicles.append(Vehicle(
                vid=len(vehicles), app=app, lane=lane, order=order, move=move,
                ini_dis_ft=position, ini_speed_fps=speed,
                outborder_ft=geometry.exit_ft(app, lane, move),
                release_s=releases[(app, lane, order)]))
    vehicles = tuple(vehicles)
    validate_vehicles(vehicles, physics, geometry)
    return Scenario(case_name=case.name, seed=seed, vehicles=vehicles,
                    geometry_hash=geometry.geometry_hash,
                    geometry_version=geometry.version)
