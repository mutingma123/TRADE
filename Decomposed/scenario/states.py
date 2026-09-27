"""Seeded initial positions and speeds inside the optimization range.

Each approach lane draws one state stream, so the leader offset and the queue
spacing of a lane stay fixed when demand, arrivals, or turning shares change.
Nested demand appends followers behind existing vehicles and never moves a
vehicle that a matched lower demand case already placed.

The minimum spacing uses the configured initial rear-end risk target. Wider
queues can extend upstream of coordinate zero; this is still shared approach
pavement and remains covered by the following checks.
"""

from __future__ import annotations

import random
import math
from dataclasses import dataclass
from statistics import NormalDist

from ..config.hashing import stream_seed
from ..config.validators import nonnegative, positive

LANES = [(app, lane) for app in (1, 2, 3, 4) for lane in (1, 2)]


@dataclass(frozen=True)
class InitialStateBounds:
    lead_setback_ft: float = 80.0
    lead_jitter_ft: float = 8.0
    queue_spacing_ft: float = 30.0
    spacing_jitter_ft: float = 6.0
    initial_speed_fps: float = 45.0

    def __post_init__(self) -> None:
        for name in ("lead_setback_ft", "queue_spacing_ft", "initial_speed_fps"):
            positive(name, getattr(self, name))
        for name in ("lead_jitter_ft", "spacing_jitter_ft"):
            nonnegative(name, getattr(self, name))


def minimum_initial_spacing_ft(physics) -> float:
    """Front-to-front spacing for the declared initial Gaussian risk ceiling.

    A follower cannot cover the initial bumper gap faster than gap / vmax.
    For independent N(0, sigma²) vehicle shifts, requiring this time to be at
    least sqrt(2) * sigma * Phi^-1(1 - p) bounds the initial contact event by p.
    Later trajectory compression remains part of the full rear-end evaluation.
    """
    headway = (-math.sqrt(2.0) * physics.initial_rear_sigma_s
               * NormalDist().inv_cdf(physics.initial_rear_probability_limit))
    return physics.vehicle_length_ft + physics.max_speed_through_fps * headway


def initial_states(seed: int, n_per_lane: int, physics,
                   bounds: InitialStateBounds = InitialStateBounds()) -> dict:
    """Initial distance and speed per (approach, lane, queue order)."""
    if bounds.initial_speed_fps > physics.max_speed_through_fps:
        raise ValueError("Initial speed exceeds the model speed cap")
    # Round the minimum upward so saved six-decimal coordinates retain the bound.
    spacing = math.ceil(max(bounds.queue_spacing_ft, minimum_initial_spacing_ft(physics))
                        * 1e6) / 1e6 + 1e-6
    states = {}
    for app, lane in LANES:
        rng = random.Random(stream_seed(seed, "state", app, lane))
        position = physics.border_ft - bounds.lead_setback_ft - rng.uniform(0.0, bounds.lead_jitter_ft)
        for order in range(n_per_lane):
            if order:
                position -= spacing + rng.uniform(0.0, bounds.spacing_jitter_ft)
            states[(app, lane, order)] = (round(position, 6), bounds.initial_speed_fps)
    return states
