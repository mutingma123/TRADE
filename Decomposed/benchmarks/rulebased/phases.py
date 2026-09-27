"""Signal phases from the geometry: opposing approaches share one green.

Two approaches oppose each other when none of their through movements share a
cell. Each such pair forms a phase, and their left turns run permissively. An
approach with no opposing partner gets a phase of its own.
"""

from __future__ import annotations

from ...geometry.cells import legal_moves
from ...geometry.intersection import APPROACHES, LANES


def _throughs(app):
    return [(app, lane, 0) for lane in LANES if 0 in legal_moves(lane)]


def opposing(geometry, first, second) -> bool:
    return not any(geometry.shared_cells(a, b)
                   for a in _throughs(first) for b in _throughs(second))


def signal_phases(geometry) -> tuple:
    """Approach groups in order of their lowest approach number."""
    phases, placed = [], set()
    for app in APPROACHES:
        if app in placed:
            continue
        partners = [other for other in APPROACHES
                    if other not in placed and other != app and opposing(geometry, app, other)]
        phase = (app, *partners[:1])
        phases.append(phase)
        placed.update(phase)
    return tuple(phases)


def phase_of(phases, app) -> tuple:
    return next(phase for phase in phases if app in phase)
