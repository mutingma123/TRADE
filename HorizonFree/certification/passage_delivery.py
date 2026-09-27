"""Independent schedule endpoints and fixed-passage checks."""

import numpy as np
from ..polynomial.bernstein import evaluate_segment, segment_at, segment_coefficients
from ..polynomial.extrema import polynomial_range

def check_passage(vehicle, pieces, geometry, routes, entry, end, speed, entry_acc, fixed_passage, tolerance, failures):
    vid = vehicle.vid
    boundary = routes[vid][0].front_entry_ft
    target = geometry.clearance_target_ft(vehicle.app, vehicle.lane, vehicle.move)
    expected_end = (fixed_passage[-1]["end_s"] if fixed_passage
                    else entry + (target - boundary) / speed)
    if abs(expected_end - end) > tolerance:
        failures.append(f"vehicle {vid}: inconsistent clearance schedule")
    for name, time, position in (("entry", entry, boundary), ("completion", end, target)):
        try:
            piece = segment_at(pieces, time)
            if abs(evaluate_segment(piece, time) - position) > tolerance:
                failures.append(f"vehicle {vid}: missed {name} position")
            expected_speed = (evaluate_segment(fixed_passage[-1], end, 1)
                              if fixed_passage and name == "completion" else speed)
            expected_acc = (evaluate_segment(fixed_passage[-1], end, 2)
                            if fixed_passage and name == "completion" else entry_acc)
            if abs(evaluate_segment(piece, time, 1) - expected_speed) > tolerance:
                failures.append(f"vehicle {vid}: missed {name} speed")
            if abs(evaluate_segment(piece, time, 2) - expected_acc) > tolerance:
                failures.append(f"vehicle {vid}: missed {name} acceleration")
        except ValueError:
            failures.append(f"vehicle {vid}: missing scheduled {name}")
    # Check the entire passage against the fixed Stage 1 motion mode.
    if fixed_passage is None:
        fixed_passage = [dict(start_s=entry, end_s=end,
                              coefficients_ft=[boundary + speed * (end - entry) * k / 5 for k in range(6)])]
    for segment in pieces:
        lo, hi = max(entry, segment["start_s"]), segment["end_s"]
        if hi <= lo + 1e-10:
            continue
        for prescribed in fixed_passage:
            shared_lo, shared_hi = max(lo, prescribed["start_s"]), min(hi, prescribed["end_s"])
            if shared_hi <= shared_lo + 1e-10:
                continue
            local = segment_coefficients(segment, shared_lo, shared_hi)
            expected = segment_coefficients(prescribed, shared_lo, shared_hi)
            lower, upper, _, _ = polynomial_range(np.asarray(local) - np.asarray(expected))
            if max(abs(lower), abs(upper)) > tolerance:
                failures.append(f"vehicle {vid}: departure from scheduled passage")
    return fixed_passage
