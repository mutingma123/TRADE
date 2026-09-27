"""DelayRecord: release, clearance, travel, free travel, and delay seconds.

Actual travel is the complete downstream rear clearance minus the release
instant. Delay is that travel minus the free travel benchmark, clamped at zero
for numerical noise, with a materially negative raw value flagged rather than
hidden. A vehicle that never clears has no delay, and the cost says unavailable
instead of using the final sampled time.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

from .freetravel import free_travel_time

NOISE_TOL = 1e-6


@dataclass(frozen=True)
class DelayRecord:
    vid: int
    release_s: float
    clearance_s: Optional[float]
    travel_s: Optional[float]
    free_travel_s: float
    delay_s: Optional[float]
    raw_delay_s: Optional[float]
    flagged_negative: bool = False


@dataclass
class DelaySummary:
    records: tuple = ()
    total_delay_s: Optional[float] = None
    mean_delay_s: Optional[float] = None
    delay_cost_usd: Optional[float] = None
    available: bool = False
    missing: tuple = field(default_factory=tuple)


def _record(vehicle, clearance, geometry, physics) -> DelayRecord:
    free = free_travel_time(vehicle, geometry, physics)
    travel = clearance.travel_s if clearance and clearance.available else None
    raw = None if travel is None else travel - free
    delay = None if raw is None else max(0.0, raw)
    return DelayRecord(vid=vehicle.vid, release_s=vehicle.release_s,
                       clearance_s=clearance.clearance_s if clearance else None,
                       travel_s=travel, free_travel_s=free, delay_s=delay, raw_delay_s=raw,
                       flagged_negative=raw is not None and raw < -NOISE_TOL)


def delay_cost(scenario, clearances, geometry, physics, pricing) -> DelaySummary:
    """Priced delay in vehicle-hours; unavailable when any vehicle is missing."""
    by_vehicle = {record.vid: record for record in clearances}
    records = tuple(_record(v, by_vehicle.get(v.vid), geometry, physics)
                    for v in scenario.vehicles)
    missing = tuple(r.vid for r in records if r.delay_s is None)
    if missing:
        return DelaySummary(records=records, available=False, missing=missing)
    total = sum(r.delay_s for r in records)
    return DelaySummary(records=records, total_delay_s=total,
                        mean_delay_s=total / len(records),
                        delay_cost_usd=pricing.delay_per_vehicle_hour * total / 3600.0,
                        available=True)
