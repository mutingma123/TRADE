"""Shared schema checks used by every configuration dataclass.

Each helper raises `ValueError` with the offending field name, so a rejected
configuration says exactly which declared setting is invalid.
"""

from __future__ import annotations

import math
from typing import Iterable


def positive(name: str, value: float) -> float:
    if not math.isfinite(value) or value <= 0:
        raise ValueError(f"{name} must be finite and positive, got {value!r}")
    return value


def nonnegative(name: str, value: float) -> float:
    if not math.isfinite(value) or value < 0:
        raise ValueError(f"{name} must be finite and nonnegative, got {value!r}")
    return value


def fraction(name: str, value: float) -> float:
    if not math.isfinite(value) or not 0.0 <= value <= 1.0:
        raise ValueError(f"{name} must lie in the unit interval, got {value!r}")
    return value


def one_of(name: str, value, allowed: Iterable) -> object:
    allowed = tuple(allowed)
    if value not in allowed:
        raise ValueError(f"{name} must be one of {allowed}, got {value!r}")
    return value
