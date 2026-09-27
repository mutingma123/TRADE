"""Bernstein coefficient certificates for continuous following gaps."""

import math
from Decomposed.geometry.following import following_pairs
from ..polynomial.bernstein import segment_coefficients
from .following_intervals import following_intervals

GAP_PIECE_S = 0.25

def add_following_constraints(model, scenario, segments, schedules, physics):
    n_gap_intervals = 0
    for leader, follower in following_pairs(scenario.vehicles):
        index = 0
        for lo, hi, gap, lead, follow in following_intervals(leader, follower, segments, schedules, physics):
            # Short pieces keep the coefficient hull within a few hundredths of a foot of the gap.
            pieces = max(1, math.ceil((hi - lo) / GAP_PIECE_S))
            for part in range(pieces):
                left, right = lo + (hi - lo) * part / pieces, lo + (hi - lo) * (part + 1) / pieces
                lead_coeff = segment_coefficients(lead, left, right)
                follow_coeff = segment_coefficients(follow, left, right)
                for k, (ahead, behind) in enumerate(zip(lead_coeff, follow_coeff)):
                    model.addConstr(ahead - behind >= gap,
                                    name=f"gap_{leader.vid}_{follower.vid}_{index}_{k}")
                index += 1
                n_gap_intervals += 1
    return n_gap_intervals
