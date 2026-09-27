"""The crossing probability of one pair, from the Gaussian mass of its union.

Each vehicle carries one independent error with the common standard deviation,
so their difference is normal with mean gamma and variance twice that of a
single error. The probability is the mass that difference places on the merged
union of arrival windows. At zero error the probability is the strict
membership indicator, evaluated without dividing by zero.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from .union import contains, merge_windows
from .windows import arrival_windows

SQRT2 = math.sqrt(2.0)


def _standard_normal_cdf(z: float) -> float:
    return 0.5 * (1.0 + math.erf(z / SQRT2))


def interval_mass(lower: float, upper: float, mean: float, scale: float) -> float:
    """Mass a normal places on one interval, computed on the stable tail."""
    if upper <= lower or scale <= 0:
        return 0.0
    low_z, high_z = (lower - mean) / scale, (upper - mean) / scale
    if low_z > 0:
        return max(0.0, _standard_normal_cdf(-low_z) - _standard_normal_cdf(-high_z))
    return max(0.0, _standard_normal_cdf(high_z) - _standard_normal_cdf(low_z))


@dataclass(frozen=True)
class PairProbability:
    first: int
    second: int
    reference_cell: int
    shared_cells: tuple
    gamma_s: float
    sigma_s: float
    union: tuple
    probability: float


def pair_probability(pair, sigma_s: float) -> PairProbability:
    """One probability per unordered conflicting-approach vehicle pair."""
    if sigma_s < 0:
        raise ValueError("The arrival error standard deviation must be nonnegative")
    union = merge_windows(arrival_windows(pair))
    if sigma_s == 0:
        value = 1.0 if contains(union, pair.gamma_s) else 0.0
    else:
        scale = SQRT2 * sigma_s
        value = sum(interval_mass(lower, upper, pair.gamma_s, scale)
                    for lower, upper in union)
        value = min(1.0, max(0.0, value))
    return PairProbability(first=pair.first, second=pair.second,
                           reference_cell=pair.reference_cell,
                           shared_cells=pair.shared_cells, gamma_s=pair.gamma_s,
                           sigma_s=sigma_s, union=union, probability=value)
