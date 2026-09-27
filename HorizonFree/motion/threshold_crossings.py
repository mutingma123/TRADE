"""Locate and cut physical motion at thresholds."""

from scipy.optimize import brentq
from ..polynomial.bernstein import evaluate_segment, restrict_coefficients

def crossing(pieces, position):
    for piece in pieces:
        lo, hi = piece['start_s'], piece['end_s']
        if evaluate_segment(piece, hi) >= position - 1e-8:
            if abs(evaluate_segment(piece, lo) - position) < 1e-8:
                return lo
            if abs(evaluate_segment(piece, hi) - position) < 1e-8:
                return hi
            return brentq(lambda t: evaluate_segment(piece, t) - position, lo, hi, xtol=1e-12)
    raise ValueError(f'Unreached position {position}')


def portion(piece, start, end):
    length = piece['end_s'] - piece['start_s']
    return dict(start_s=start, end_s=end, coefficients_ft=restrict_coefficients(
        piece['coefficients_ft'], (start - piece['start_s']) / length,
        (end - piece['start_s']) / length))


def _finish(motion, target, cap):
    if motion.x < target - 1e-8:
        motion.add((target - motion.x) / cap)
    finish = crossing(motion.pieces, target)
    motion.pieces = [portion(p, p['start_s'], min(p['end_s'], finish)) for p in motion.pieces
                     if p['start_s'] < finish - 1e-9]
    return finish
