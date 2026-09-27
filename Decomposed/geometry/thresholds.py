"""OccupancyThreshold: front entry and rear exit distances per route cell.

Full vehicle occupancy applies on through and turning paths alike: a cell is
occupied from the moment the front passes the cell's lower bound until the rear
clears its upper bound, so the rear exit distance carries one vehicle length.
Result recovers occupancy intervals by interpolating the trajectory at exactly
these two distances.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class OccupancyThreshold:
    cell: int
    front_entry_ft: float
    rear_exit_ft: float

    def __post_init__(self) -> None:
        if self.rear_exit_ft <= self.front_entry_ft:
            raise ValueError(f"Cell {self.cell} has a nonpositive occupancy span")


def thresholds_for(route, vehicle_length_ft: float) -> tuple:
    """Occupancy thresholds for one route, in the route's own cell order."""
    if vehicle_length_ft <= 0:
        raise ValueError("Vehicle length must be positive for full body occupancy")
    return tuple(
        OccupancyThreshold(cell=c.cell, front_entry_ft=c.lb_ft,
                           rear_exit_ft=c.ub_ft + vehicle_length_ft)
        for c in route.cells
    )
