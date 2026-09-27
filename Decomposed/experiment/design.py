"""The eight settings of the synchronized and random vehicle ladders.

Each setting fixes the demand, the arrival pattern, the turning share, the
number of seeds the design allows, and the declared physics horizon that the
rule-based controllers report against. The horizon-free model never reads it.
"""

from dataclasses import dataclass, replace

from ..config.study import StudyConfig
from ..scenario.case import TrafficCase


@dataclass(frozen=True)
class Setting:
    name: str
    n_per_lane: int
    arrival: str
    turn_fraction: float
    n_seeds: int
    horizon_s: float

    def case(self):
        return TrafficCase(self.name, self.n_per_lane, self.arrival, self.turn_fraction)

    def config(self, base=None):
        base = base or StudyConfig()
        return replace(base, physics=replace(base.physics, horizon_s=self.horizon_s))


SETTINGS = {row.name: row for row in (
    Setting(f"v{8 * count}_{arrival}_50", count, arrival, 0.5, seeds, horizon)
    for count, seeds, horizon in ((2, 30, 30), (3, 30, 30), (4, 20, 50), (5, 20, 70))
    for arrival in ("synchronized", "random"))}


def setting_named(name):
    if name not in SETTINGS:
        raise ValueError(f"Unknown setting {name!r}; choose from {sorted(SETTINGS)}")
    return SETTINGS[name]
