"""Vehicle: one identity, one route, one release state.

The identifier is stable across ceilings and controllers. `release_s` is the
instant modeled motion starts; before it the vehicle holds its initial
distance at zero speed, and no trajectory state is reported for it.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass


@dataclass(frozen=True)
class Vehicle:
    vid: int
    app: int
    lane: int
    order: int
    move: int
    ini_dis_ft: float
    ini_speed_fps: float
    outborder_ft: float
    release_s: float = 0.0

    def __post_init__(self) -> None:
        values = (self.ini_dis_ft, self.ini_speed_fps, self.outborder_ft, self.release_s)
        if not all(math.isfinite(v) for v in values):
            raise ValueError(f"Vehicle {self.vid} has a nonfinite initial state")
        if self.ini_speed_fps < 0 or self.release_s < 0:
            raise ValueError(f"Vehicle {self.vid} has a negative speed or release time")
        if self.order < 0:
            raise ValueError(f"Vehicle {self.vid} has a negative queue order")

    @property
    def is_turning(self) -> bool:
        return self.move != 0

    def to_dict(self) -> dict:
        return asdict(self)
