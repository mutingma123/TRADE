"""Expected colliding pair count and the conditional crossing cost.

The expected count is the sum of the pair probabilities. Expectation is linear,
so pair independence is not required, but the sum is a count and not the
probability that any collision occurs. The cost multiplies that count by the
nonnegative per-pair price Pricing supplies, and it is conditional on the
declared error model and the planned schedule.
"""

from __future__ import annotations

from dataclasses import dataclass

from .pairs import conflicting_pairs
from .probability import pair_probability


@dataclass(frozen=True)
class CrossingCost:
    sigma_s: float
    price_per_pair: float
    eligible_pairs: int
    expected_pairs: float
    cost: float
    probabilities: tuple


def expected_pair_count(intervals, scenario, sigma_s: float) -> tuple:
    """(probabilities, expected count); an empty pair set gives zero."""
    pairs = conflicting_pairs(intervals, scenario)
    probabilities = tuple(pair_probability(pair, sigma_s) for pair in pairs)
    return probabilities, sum(p.probability for p in probabilities)


def crossing_cost(intervals, scenario, sigma_s: float, price_per_pair: float) -> CrossingCost:
    """Conditional expected crossing loss for one schedule at one error width."""
    if price_per_pair < 0:
        raise ValueError("The crossing pair cost must be nonnegative")
    probabilities, expected = expected_pair_count(intervals, scenario, sigma_s)
    return CrossingCost(sigma_s=sigma_s, price_per_pair=price_per_pair,
                        eligible_pairs=len(probabilities), expected_pairs=expected,
                        cost=price_per_pair * expected, probabilities=probabilities)
