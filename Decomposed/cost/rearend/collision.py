"""Expected rear-end collisions: the crossing Gaussian mass on a following margin.

Each vehicle carries one independent arrival error, so the follower minus
leader difference is normal with twice the single variance, exactly as for a
crossing pair. A follower in the same lane cannot get ahead of its leader
without meeting it, so its collision window is one sided: every difference
below minus the pair's smallest margin. The probability is the Gaussian mass
the crossing evaluator places on that window, and zero width gives the strict
nominal indicator. The expected count sums the pair probabilities and the cost
prices each expected collision at the rear unit cost. A pair with no comparable
pavement is reported as not compared, never as a zero.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional

from ...stochastic_timing_error.probability import interval_mass
from .pairs import coverage_limits

SQRT2 = math.sqrt(2.0)


@dataclass(frozen=True)
class RearProbability:
    leader: int
    follower: int
    min_headway_s: Optional[float]
    sigma_s: float
    probability: Optional[float]


@dataclass(frozen=True)
class RearCost:
    sigma_s: float
    price_per_pair: float
    following_pairs: int
    compared_pairs: int
    expected_pairs: float
    cost: float
    min_headway_s: Optional[float]
    probabilities: tuple
    coverage_limits: tuple = ()


def rear_probability(margin, sigma_s: float) -> RearProbability:
    """One collision probability per following pair, from its smallest margin."""
    if sigma_s < 0:
        raise ValueError("The arrival error standard deviation must be nonnegative")
    rear = margin.rear
    if not margin.compared:
        value = None
    elif sigma_s == 0:
        value = 1.0 if margin.min_headway_s < 0 else 0.0
    else:
        value = interval_mass(-math.inf, -margin.min_headway_s, 0.0, SQRT2 * sigma_s)
        value = min(1.0, max(0.0, value))
    return RearProbability(leader=rear.leader, follower=rear.follower,
                           min_headway_s=margin.min_headway_s, sigma_s=sigma_s,
                           probability=value)


def rear_cost(margins, sigma_s: float, price_per_pair: float) -> RearCost:
    """Conditional expected rear-end collision loss for one schedule at one width."""
    if price_per_pair < 0:
        raise ValueError("The rear-end pair cost must be nonnegative")
    probabilities = tuple(rear_probability(margin, sigma_s) for margin in margins)
    compared = [p for p in probabilities if p.probability is not None]
    expected = sum(p.probability for p in compared)
    values = [p.min_headway_s for p in compared]
    return RearCost(sigma_s=sigma_s, price_per_pair=price_per_pair,
                    following_pairs=len(probabilities), compared_pairs=len(compared),
                    expected_pairs=expected, cost=price_per_pair * expected,
                    min_headway_s=min(values) if values else None,
                    probabilities=probabilities,
                    coverage_limits=coverage_limits(tuple(m.rear for m in margins)))
