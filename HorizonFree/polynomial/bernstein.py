"""Bernstein derivatives, restrictions, and evaluation."""



def derivative_coefficients(coefficients, duration_s, order=1):
    """Bernstein coefficients of a physical-time derivative; also works on LP expressions."""
    result = list(coefficients)
    for _ in range(order):
        degree = len(result) - 1
        result = [degree * (right - left) / duration_s
                  for left, right in zip(result, result[1:])]
    return result


def _split(coefficients, fraction):
    row = list(coefficients)
    left, right = [row[0]], [row[-1]]
    while len(row) > 1:
        row = [(1.0 - fraction) * a + fraction * b for a, b in zip(row, row[1:])]
        left.append(row[0])
        right.append(row[-1])
    return left, right[::-1]


def restrict_coefficients(coefficients, lower, upper):
    """Exact de Casteljau restriction from [0,1] to [lower,upper]."""
    if lower < -1e-8 or upper > 1.0 + 1e-8 or upper < lower:
        raise ValueError(f"Invalid Bernstein restriction [{lower}, {upper}]")
    lower, upper = max(0.0, lower), min(1.0, upper)
    if lower >= 1.0:
        return [coefficients[-1]] * len(coefficients)
    _, right = _split(coefficients, lower)
    left, _ = _split(right, (upper - lower) / (1.0 - lower))
    return left


def segment_coefficients(segment, start_s, end_s):
    duration = segment["end_s"] - segment["start_s"]
    return restrict_coefficients(segment["coefficients_ft"],
                                 (start_s - segment["start_s"]) / duration,
                                 (end_s - segment["start_s"]) / duration)


def segment_at(segments, time_s):
    for segment in segments:
        if segment["start_s"] - 1e-8 <= time_s <= segment["end_s"] + 1e-8:
            return segment
    raise ValueError(f"Time {time_s} lies outside the vehicle trajectory")


def evaluate_segment(segment, time_s, order=0):
    duration = segment["end_s"] - segment["start_s"]
    fraction = min(1.0, max(0.0, (time_s - segment["start_s"]) / duration))
    coefficients = derivative_coefficients(segment["coefficients_ft"], duration, order)
    return float(_split(coefficients, fraction)[0][-1])
