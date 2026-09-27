"""Release times for the synchronized and random arrival patterns.

Synchronized releases every vehicle at once, so the initial position jitter is
the only staggering and the case is the stress reference. Random uses seeded
exponential headways per lane. Every release is rounded up onto the common
model time grid, and a follower never precedes its leader.
"""

from __future__ import annotations

import random

from ..config.hashing import stream_seed

LANES = [(app, lane) for app in (1, 2, 3, 4) for lane in (1, 2)]


def _raw_release(case, seed: int, app: int, lane: int, order: int) -> float:
    if case.arrival == "synchronized":
        return 0.0
    rng = random.Random(stream_seed(seed, "arrival", app, lane))
    return sum(rng.expovariate(1.0 / case.random_mean_headway_s)
               for _ in range(order + 1))


def release_times(case, seed: int, physics) -> dict:
    """Release time per (approach, lane, queue order), on the model grid."""
    releases = {}
    for app, lane in LANES:
        previous = 0.0
        for order in range(case.n_per_lane):
            raw = _raw_release(case, seed, app, lane, order)
            gridded = physics.step_of(max(raw, previous)) * physics.dt_s
            releases[(app, lane, order)] = round(gridded, 8)
            previous = releases[(app, lane, order)]
    return releases
