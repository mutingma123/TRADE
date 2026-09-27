"""Exact legal turning shares with matched seeds.

Turning vehicles are allocated by lane type: lane 1 turns right, lane 2 turns
left, which is what the published geometry permits. A share of one half over
eight lanes therefore places two right turns and two left turns per queue
position, and the realized share equals the requested share exactly.
"""

from __future__ import annotations

import random

from ..config.hashing import stream_seed
from ..geometry.cells import legal_moves

APPROACHES = (1, 2, 3, 4)
LANES = (1, 2)


def turning_apps(seed: int, lane: int, order: int, turn_fraction: float) -> frozenset:
    """Approaches whose vehicle at this queue position turns from this lane."""
    count = round(len(APPROACHES) * turn_fraction)
    if abs(len(APPROACHES) * turn_fraction - count) > 1e-9:
        raise ValueError("The turning share must place a whole number of turns per lane")
    apps = list(APPROACHES)
    random.Random(stream_seed(seed, "movement", lane, order)).shuffle(apps)
    return frozenset(apps[:count])


def allocate_turns(seed: int, order: int, turn_fraction: float) -> dict:
    """Movement for every (approach, lane) at one queue position."""
    chosen = {lane: turning_apps(seed, lane, order, turn_fraction) for lane in LANES}
    moves = {}
    for app in APPROACHES:
        for lane in LANES:
            turn_move = legal_moves(lane)[1]
            moves[(app, lane)] = turn_move if app in chosen[lane] else 0
    return moves

