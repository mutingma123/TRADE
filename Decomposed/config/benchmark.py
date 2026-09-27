"""BenchmarkConfig: the parameters of the three rule-based controllers.

The stop dwell and signal timings are declared starting assumptions, not
measured values, so each one is recorded with its trajectory.
`queue_gap_ft` is the standstill bumper gap between vehicles queued at a stop
line, likewise an assumption.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass

from .validators import positive


@dataclass(frozen=True)
class BenchmarkConfig:
    stop_dwell_s: float = 1.0
    min_green_s: float = 10.0
    max_green_s: float = 40.0
    gap_out_s: float = 3.0
    intergreen_s: float = 3.0
    reservation_guard_s: float = 0.1
    queue_gap_ft: float = 5.0
    max_reservation_wait_s: float = 600.0

    def __post_init__(self) -> None:
        for name in ("stop_dwell_s", "min_green_s", "max_green_s", "gap_out_s",
                     "intergreen_s", "reservation_guard_s", "queue_gap_ft",
                     "max_reservation_wait_s"):
            positive(name, getattr(self, name))
        if self.max_green_s <= self.min_green_s:
            raise ValueError("max_green_s must exceed min_green_s")

    def to_dict(self) -> dict:
        return asdict(self)
