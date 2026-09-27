"""CellInterval: the occupancy window of one vehicle in one conflict cell.

Entry is the instant the front passes the cell's front entry distance and exit
the instant the rear passes its rear exit distance, both interpolated at the
geometry thresholds. A cell whose thresholds the trajectory does not bracket
yields a missing interval, which downstream evaluators must treat as absent
rather than as zero exposure.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional

from .interpolation import crossing_time


@dataclass(frozen=True)
class CellInterval:
    vid: int
    app: int
    lane: int
    move: int
    cell: int
    entry_s: Optional[float]
    exit_s: Optional[float]
    geometry_hash: str = ""

    @property
    def available(self) -> bool:
        return self.entry_s is not None and self.exit_s is not None

    @property
    def duration_s(self) -> Optional[float]:
        """Occupancy duration; must be finite and positive to be usable."""
        if not self.available:
            return None
        return self.exit_s - self.entry_s


def cell_intervals(trajectory, scenario, geometry) -> tuple:
    """Occupancy intervals for every vehicle and every cell of its route."""
    rows = []
    for vehicle in scenario.vehicles:
        vid = vehicle.vid
        points = trajectory.points(vid)
        for threshold in geometry.thresholds(vehicle.app, vehicle.lane, vehicle.move):
            rows.append(CellInterval(
                vid=vid, app=vehicle.app, lane=vehicle.lane, move=vehicle.move,
                cell=threshold.cell,
                entry_s=crossing_time(points, threshold.front_entry_ft),
                exit_s=crossing_time(points, threshold.rear_exit_ft),
                geometry_hash=geometry.geometry_hash))
    return tuple(rows)
