"""PhysicsConfig: units, vehicle limits, geometry identifiers, and time grid.

Every controller and every solve sees this one physical definition, so a
benchmark trajectory and an optimized trajectory are comparable by
construction. The horizon is a modeling choice and stays explicit even when
solving is unlimited.
"""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass

from .validators import positive, nonnegative


@dataclass(frozen=True)
class PhysicsConfig:
    dt_s: float = 0.1
    horizon_s: float = 30.0
    border_ft: float = 600.0
    after_length_ft: float = 100.0
    vehicle_length_ft: float = 10.0
    safe_buffer_ft: float = 60.0
    max_acc_fps2: float = 10.0
    max_dec_fps2: float = -15.0
    max_acc_change_fps2: float = 3.0
    min_speed_fps: float = 0.0
    max_speed_through_fps: float = 60.0
    max_speed_turn_fps: float = 40.0
    crossing_window_s: float = 3.0
    n_cells: int = 16
    geometry_version: str = "16cell-v1"
    initial_rear_sigma_s: float = 1.0
    initial_rear_probability_limit: float = 0.001

    def __post_init__(self) -> None:
        for name in ("dt_s", "horizon_s", "border_ft", "after_length_ft",
                     "vehicle_length_ft", "max_acc_fps2", "crossing_window_s",
                     "max_speed_turn_fps", "max_acc_change_fps2", "max_speed_through_fps",
                     "initial_rear_sigma_s"):
            positive(name, getattr(self, name))
        nonnegative("safe_buffer_ft", self.safe_buffer_ft)
        nonnegative("min_speed_fps", self.min_speed_fps)
        if self.max_dec_fps2 >= 0:
            raise ValueError("max_dec_fps2 must be negative")
        if self.max_speed_turn_fps > self.max_speed_through_fps:
            raise ValueError("The turning cap cannot exceed the through cap")
        if self.n_cells != 16:
            raise ValueError("Only the published 16-cell partition is packaged")
        if not 0.0 < self.initial_rear_probability_limit < 0.5:
            raise ValueError("initial_rear_probability_limit must lie strictly between 0 and 0.5")

    @property
    def horizon_steps(self) -> int:
        """Number of time steps; the state grid holds one more sample."""
        return round(self.horizon_s / self.dt_s)

    def step_of(self, time_s: float) -> int:
        """Round a time upward onto the common model grid."""
        return math.ceil(round(time_s / self.dt_s, 6))

    def to_dict(self) -> dict:
        return asdict(self)
