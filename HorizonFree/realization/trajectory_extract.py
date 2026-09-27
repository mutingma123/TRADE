"""Extract polynomial trajectories and common sampled interchange records."""

import numpy as np
from Decomposed.result.trajectory.state import State, Trajectory
from ..polynomial.bernstein import segment_at, evaluate_segment
from .approach_constraints import DEGREE

def extract_stage2(stage, scenario, geometry, physics, label="horizon-free", sample_dt_s=0.02):
    """Return (serializable polynomials, canonical sampled Trajectory).

    Sampling is solely for shared cost/interchange routines. Continuous safety
    is certified on the polynomial representation, never on sampled states.
    All cell events, polynomial joins, and exact completion times are included.
    """
    if sample_dt_s <= 0:
        raise ValueError("sample_dt_s must be positive")
    if stage.model.SolCount <= 0:
        raise ValueError("Cannot extract Stage 2 without an incumbent")
    result = dict(basis="Bernstein", degree=DEGREE, sample_dt_s=sample_dt_s, vehicles={})
    states = {}
    for vehicle in scenario.vehicles:
        vid = vehicle.vid
        pieces = [dict(start_s=seg["start_s"], end_s=seg["end_s"],
                       coefficients_ft=[float(c.X) if hasattr(c, "X") else float(c)
                                        for c in seg["coefficients_ft"]])
                  for seg in stage.segments[vid]]
        schedule = stage.schedules[vid]
        result["vehicles"][vid] = dict(schedule, segments=pieces)
        release, end = pieces[0]["start_s"], pieces[-1]["end_s"]
        times = set(float(t) for t in np.arange(release, end, sample_dt_s))
        times.update(t for seg in pieces for t in (seg["start_s"], seg["end_s"]))
        from ..polynomial.extrema import first_crossing
        for threshold in geometry.thresholds(vehicle.app, vehicle.lane, vehicle.move):
            for position in (threshold.front_entry_ft, threshold.rear_exit_ft):
                event_time = first_crossing(pieces, position)
                if event_time is not None:
                    times.add(event_time)
        rows = []
        for time in sorted(times):
            piece = segment_at(pieces, time)
            x, v, a = (evaluate_segment(piece, time, order) for order in range(3))
            if abs(time - end) <= 1e-10:
                x = geometry.clearance_target_ft(vehicle.app, vehicle.lane, vehicle.move)
            rows.append(State(vid, float(time), x, v, a, int(x < physics.border_ft)))
        states[vid] = rows
    trajectory = Trajectory(label=label, scenario_hash=scenario.scenario_hash,
                            geometry_hash=geometry.geometry_hash, states=states,
                            availability="complete", source="horizon-free-polynomial", dt_s=sample_dt_s)
    return result, trajectory
