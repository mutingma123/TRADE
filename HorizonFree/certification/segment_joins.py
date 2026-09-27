"""Check polynomial joins with propagated float64 derivative roundoff."""

from ..polynomial.bernstein import evaluate_segment
from ..polynomial.extrema import rounding_allowance


def check_joins(vid, pieces, tolerance, failures):
    largest_allowance = 0.0
    for left, right in zip(pieces, pieces[1:]):
        if abs(left["end_s"] - right["start_s"]) > tolerance:
            failures.append(f"vehicle {vid}: segment time gap or overlap")
        for order in range(3):
            before = evaluate_segment(left, left["end_s"], order)
            after = evaluate_segment(right, right["start_s"], order)
            allowance = sum(rounding_allowance(piece["coefficients_ft"],
                            piece["end_s"] - piece["start_s"], order)
                            for piece in (left, right))
            largest_allowance = max(largest_allowance, allowance)
            if abs(before - after) > tolerance + allowance:
                failures.append(f"vehicle {vid}: derivative {order} discontinuity at {right['start_s']}")
    return largest_allowance
