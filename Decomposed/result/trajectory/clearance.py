"""ClearanceRecord: release and complete downstream rear clearance per vehicle.

Delay measures travel from the release instant to this clearance instant, so
both are recorded together. A vehicle that never reaches the clearance target
has no clearance time, and the record says so instead of reporting the final
sampled time.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .interpolation import crossing_time


@dataclass(frozen=True)
class ClearanceRecord:
    vid: int
    release_s: float
    clearance_s: Optional[float]
    target_ft: float

    @property
    def available(self) -> bool:
        return self.clearance_s is not None

    @property
    def travel_s(self) -> Optional[float]:
        if not self.available:
            return None
        return self.clearance_s - self.release_s


def clearance_of(trajectory, scenario, geometry) -> tuple:
    """One clearance record per vehicle in the scenario."""
    records = []
    for vehicle in scenario.vehicles:
        target = geometry.clearance_target_ft(vehicle.app, vehicle.lane, vehicle.move)
        records.append(ClearanceRecord(
            vid=vehicle.vid, release_s=vehicle.release_s,
            clearance_s=crossing_time(trajectory.points(vehicle.vid), target),
            target_ft=target))
    return tuple(records)


def availability_of(records, expected: int) -> str:
    """Complete only when every expected vehicle cleared the same target."""
    if not records or all(not r.available for r in records):
        return "missing"
    if len(records) < expected or any(not r.available for r in records):
        return "partial"
    return "complete"
