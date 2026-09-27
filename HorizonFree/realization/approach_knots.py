"""Vehicle-local approach knots preserve all witness switches."""

import numpy as np

MIN_KNOT_GAP_S = 0.1

def approach_knots(vehicle, schedule, release, entry, n_segments):
    # Witness breakpoints and the turning-cap instant are required knots.
    # Uniform knots too close to them would create ill-conditioned slivers.
    cap_time = release + 0.5
    knots = {release, entry}
    knots.update(float(t) for t in schedule.get("approach_breakpoints_s", ())
                 if release < float(t) < entry)
    if vehicle.is_turning and release < cap_time < entry:
        knots.add(cap_time)
    for t in np.linspace(release, entry, n_segments + 1)[1:-1]:
        if min(abs(float(t) - k) for k in knots) >= MIN_KNOT_GAP_S:
            knots.add(float(t))
    knots = sorted(knots)
    return knots, cap_time
