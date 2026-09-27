"""Compatibility of adjacent vehicles and motion modes."""

import math
from .lane_gap import witness, gap_margin, _smallest, _largest, TIME_TOL_S

MARGIN_FT = 0.25
GRID_S = 0.05
STATIONARY_S = 20.0

def pair_rules(leader, follower, lead_modes, follow_modes, physics, margin=MARGIN_FT):
    """Compatibility rules for one adjacent same-lane pair, keyed by mode indices."""
    def ok(i, wait_a, j, wait_b):
        lead, follow = witness(lead_modes[i], wait_a), witness(follow_modes[j], wait_b)
        return gap_margin(leader, follower, lead, follow, physics) >= margin

    def stopping(mode):
        return mode['wait_start_s'] is not None

    forbidden, min_follow_wait, max_lead_wait, offsets = [], {}, {}, {}
    for i, lead_mode in enumerate(lead_modes):
        for j, follow_mode in enumerate(follow_modes):
            lead_stops, follow_stops = stopping(lead_mode), stopping(follow_mode)
            if not lead_stops and not follow_stops:
                if not ok(i, 0.0, j, 0.0):
                    forbidden.append((i, j))
            elif not lead_stops:
                # The follower waits until the leader has cleared at the latest.
                upper = max(1.0, lead_mode['entry_s'] + lead_mode['duration_s']
                            - follow_mode['wait_start_s'] + 1.0)
                wait = _smallest(lambda w: ok(i, 0.0, j, w), upper)
                if wait is None:
                    forbidden.append((i, j))
                elif wait > 0.0:
                    min_follow_wait[i, j] = wait + TIME_TOL_S
            elif not follow_stops:
                # A leader departing after the follower's clearance must fail.
                upper = max(1.0, follow_mode['entry_s'] + follow_mode['duration_s']
                            - lead_mode['wait_start_s'] + 1.0)
                wait = _largest(lambda w: ok(i, w, j, 0.0), upper)
                if wait is None:
                    forbidden.append((i, j))
                else:
                    max_lead_wait[i, j] = max(0.0, wait - TIME_TOL_S)
            else:
                # Beyond ``settle`` the leader departs after the follower has
                # stopped, and only the departure offset matters. The last
                # point is far enough out for the follower's wait to bind.
                settle = max(0.0, follow_mode['wait_start_s'] - lead_mode['wait_start_s'])
                grid = [k * GRID_S for k in range(int(math.ceil(settle / GRID_S)) + 1)]
                grid.extend([settle + 0.5, settle + STATIONARY_S])
                required, feasible = -(settle + STATIONARY_S), True
                for wait_a in grid:
                    upper = max(1.0, wait_a + lead_mode['entry_s'] + lead_mode['duration_s']
                                - follow_mode['wait_start_s'] + 1.0)
                    wait_b = _smallest(lambda w: ok(i, wait_a, j, w), upper)
                    if wait_b is None:
                        feasible = False
                        break
                    if wait_b > 0.0:
                        required = max(required, wait_b - wait_a)
                if not feasible:
                    forbidden.append((i, j))
                else:
                    offsets[i, j] = required + 2 * TIME_TOL_S
    return dict(forbidden=forbidden, min_follow_wait=min_follow_wait,
                max_lead_wait=max_lead_wait, offsets=offsets, margin_ft=margin)
