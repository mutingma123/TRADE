"""TrafficCase: one ladder setting's demand, arrival pattern, and turning share.

Synchronized arrivals release every vehicle at once; random arrivals draw
seeded exponential headways per lane with the declared mean, a configurable
design assumption rather than a measured value.
"""

from __future__ import annotations

from dataclasses import dataclass

from ..config.validators import fraction, one_of, positive

ARRIVAL_PATTERNS = ("synchronized", "random")


@dataclass(frozen=True)
class TrafficCase:
    name: str
    n_per_lane: int = 1
    arrival: str = "synchronized"
    turn_fraction: float = 0.5
    random_mean_headway_s: float = 2.0

    def __post_init__(self) -> None:
        one_of("arrival", self.arrival, ARRIVAL_PATTERNS)
        fraction("turn_fraction", self.turn_fraction)
        positive("random_mean_headway_s", self.random_mean_headway_s)
        if not isinstance(self.n_per_lane, int) or not 1 <= self.n_per_lane <= 8:
            raise ValueError("Demand levels are one through eight vehicles per lane")
