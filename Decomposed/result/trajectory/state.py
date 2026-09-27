"""State and Trajectory: the shared schema and its availability label.

A trajectory is complete when every vehicle in the scenario has states from its
release through the clearance target, partial when some vehicle stops short,
and missing when no states exist at all.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

AVAILABILITY = ("missing", "partial", "complete")


@dataclass(frozen=True)
class State:
    vid: int
    time_s: float
    position_ft: float
    speed_fps: float
    acc_fps2: float
    before_border: int = 1

    def __post_init__(self) -> None:
        values = (self.time_s, self.position_ft, self.speed_fps, self.acc_fps2)
        if not all(math.isfinite(v) for v in values):
            raise ValueError(f"Vehicle {self.vid} has a nonfinite state at {self.time_s}")


@dataclass
class Trajectory:
    label: str
    scenario_hash: str
    geometry_hash: str
    states: dict = field(default_factory=dict)
    availability: str = "missing"
    source: str = ""
    dt_s: float = 0.1

    def __post_init__(self) -> None:
        if self.availability not in AVAILABILITY:
            raise ValueError(f"Unknown availability {self.availability!r}")

    def points(self, vid: int) -> list:
        """States of one vehicle in increasing time order."""
        return self.states.get(vid, [])

    def vehicle_ids(self) -> tuple:
        return tuple(sorted(self.states))
