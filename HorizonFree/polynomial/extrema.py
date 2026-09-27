"""Independent polynomial extrema and threshold roots."""

import math
import numpy as np
from numpy.polynomial import Polynomial

def power_polynomial(coefficients):
    """Convert Bernstein coefficients on [0,1] to ascending monomial powers."""
    coefficients = np.asarray(coefficients, dtype=float)
    degree = len(coefficients) - 1
    powers = np.zeros(degree + 1)
    for k, coefficient in enumerate(coefficients):
        for exponent in range(k, degree + 1):
            powers[exponent] += (coefficient * math.comb(degree, k)
                                * math.comb(degree - k, exponent - k)
                                * (-1) ** (exponent - k))
    return Polynomial(powers).trim(1e-11)


def _real_unit_roots(poly):
    if poly.degree() <= 0:
        return []
    return [float(min(1.0, max(0.0, root.real))) for root in poly.roots()
            if abs(root.imag) <= 1e-7 and -1e-9 <= root.real <= 1.0 + 1e-9]


def polynomial_range(coefficients):
    """Minimum and maximum over the entire unit interval, including stationary roots."""
    poly = power_polynomial(coefficients)
    points = [0.0, 1.0] + _real_unit_roots(poly.deriv())
    values = [(float(poly(t)), float(t)) for t in points]
    minimum = min(values)
    maximum = max(values)
    return minimum[0], maximum[0], minimum[1], maximum[1]


def rounding_allowance(coefficients, duration, order):
    """Float64 rounding bound on a derivative formed from position coefficients.

    Each derivative takes a difference of neighbouring coefficients and divides
    by the duration, so the error grows as falling(5, k) 2^k max|c| u / d^k.
    The factor 4 covers the later basis conversion and root evaluation.
    """
    if order == 0:
        return 0.0
    scale = max(abs(float(c)) for c in coefficients)
    return 4.0 * math.perm(len(coefficients) - 1, order) * 2 ** order * scale * np.finfo(float).eps / duration ** order


def first_crossing(segments, target, tolerance=1e-5):
    """First threshold root of the complete polynomial pass, without interpolation."""
    for segment in segments:
        poly = power_polynomial(segment["coefficients_ft"]) - target
        roots = _real_unit_roots(poly)
        if abs(poly(0.0)) <= tolerance:
            roots.append(0.0)
        if abs(poly(1.0)) <= tolerance:
            roots.append(1.0)
        if roots:
            return float(segment["start_s"] + min(roots) * (segment["end_s"] - segment["start_s"]))
    return None
