"""Independent float64 polynomial validation of each delivered trajectory."""

import math
from ..polynomial.bernstein import evaluate_segment
from .derivative_bounds import check_derivatives
from .passage_delivery import check_passage
from .event_delivery import check_events
from .pair_separation import check_pairs
from .segment_joins import check_joins

def validate_continuous(polynomials, scenario, geometry, physics, routes, schedules, *, tolerance=1e-5):
    """Certify dynamics, event delivery, following, and full-body cell exclusion."""
    failures, segments, extrema = [], {}, {}
    largest_allowance = largest_join_allowance = 0.0
    raw = polynomials.get("vehicles", {})
    intervals = []
    for vehicle in scenario.vehicles:
        vid = vehicle.vid
        record = raw.get(vid, raw.get(str(vid)))
        if not record or not record.get("segments"):
            failures.append(f"vehicle {vid}: missing polynomial trajectory")
            continue
        pieces = record["segments"]
        malformed = False
        for segment in pieces:
            values = [segment.get("start_s", math.nan), segment.get("end_s", math.nan)]
            values.extend(segment.get("coefficients_ft", []))
            if (len(segment.get("coefficients_ft", [])) != 6
                    or not all(math.isfinite(float(x)) for x in values)
                    or segment["end_s"] <= segment["start_s"]):
                failures.append(f"vehicle {vid}: malformed polynomial segment")
                malformed = True
        if malformed:
            continue
        segments[vid] = pieces
        schedule = schedules[vid]
        entry, end = (float(schedule[key]) for key in ("entry_s", "exit_s"))
        fixed_passage = schedule.get("passage_segments")
        speed = float(schedule.get("entry_speed_fps", schedule.get("passage_speed_fps", 0.0)))
        entry_acc = float(schedule.get("entry_acc_fps2", 0.0))
        if speed < 0 or (speed == 0 and not fixed_passage) or not all(math.isfinite(x) for x in (entry, end, speed, entry_acc)):
            failures.append(f"vehicle {vid}: invalid schedule")
            continue
        if abs(pieces[0]["start_s"] - vehicle.release_s) > tolerance:
            failures.append(f"vehicle {vid}: incorrect release time")
        if abs(pieces[-1]["end_s"] - end) > tolerance:
            failures.append(f"vehicle {vid}: incorrect scheduled completion time")
        for order, expected in ((0, vehicle.ini_dis_ft), (1, vehicle.ini_speed_fps)):
            actual = evaluate_segment(pieces[0], pieces[0]["start_s"], order)
            if abs(actual - expected) > tolerance:
                failures.append(f"vehicle {vid}: incorrect initial derivative {order}")
        allowance = check_joins(vid, pieces, tolerance, failures)
        largest_join_allowance = max(largest_join_allowance, allowance)
        largest_allowance = max(largest_allowance, allowance)
        extrema[vid], allowance = check_derivatives(vehicle, pieces, physics, tolerance, failures)
        largest_allowance = max(largest_allowance, allowance)
        fixed_passage = check_passage(vehicle, pieces, geometry, routes, entry, end, speed, entry_acc, fixed_passage, tolerance, failures)
        check_events(vid, pieces, routes, schedule, fixed_passage, tolerance, failures, intervals)
    minimum_gap_margin, gap_count, minimum_cell_separation = check_pairs(scenario, segments, schedules, physics, intervals, tolerance, failures)
    return dict(valid=not failures, failures=failures,
                method="continuous polynomial extrema and threshold roots (float64)",
                tolerance=tolerance, largest_rounding_allowance=largest_allowance,
                largest_join_rounding_allowance=largest_join_allowance,
                extrema=extrema, checked_following_intervals=gap_count,
                minimum_gap_margin_ft=(minimum_gap_margin if math.isfinite(minimum_gap_margin) else None),
                minimum_cell_separation_s=(minimum_cell_separation if math.isfinite(minimum_cell_separation) else None),
                occupancy_intervals=intervals,
                coverage_limits=["No downstream merge geometry for distinct movements.",
                                 "Following is checked during the pair's overlapping recorded lifetimes.",
                                 "Initial acceleration is free; acceleration is continuous from release onward."])
