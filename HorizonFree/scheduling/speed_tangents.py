"""Certified lower approximation of capped approach speed."""



def tangent_speeds(upper):
    speeds = [0.0, min(0.5, upper)]
    while speeds[-1] * 1.15 < upper - 1e-9:
        speeds.append(speeds[-1] * 1.15)
    if speeds[-1] < upper - 1e-9:
        speeds.append(upper)
    return speeds


def tangent_error(speeds):
    # Maximum absolute error for a reciprocal value between successive knots.
    return max((b - a) ** 2 / (2 * (a + b)) for a, b in zip(speeds, speeds[1:]))
