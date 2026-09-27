"""Realize one Stage 1 schedule: the Stage 2 LP, continuous validation, and common pricing."""

import time
from pathlib import Path

from Decomposed.result.artifacts import write_json
from Decomposed.solve.quiet import solve_quiet
from HorizonFree.certification.continuous_report import validate_continuous
from HorizonFree.metrics import comparable_cost, continuous_acceleration_variation
from HorizonFree.realization.stage2_model import build_stage2
from HorizonFree.realization.trajectory_extract import extract_stage2
from trade.execution.realized_measures import event_fidelity, realized_metrics
from trade.records.solver_records import solve_row


def realize(scene, geometry, physics, solver, routes, schedules, folder, label):
    """Stage 2 on the delivered schedule, then continuous validation and common pricing."""
    out = dict(valid=False)
    started = time.perf_counter()
    with build_stage2(scene, geometry, physics, routes, schedules) as stage:
        out["stage2_build_s"] = time.perf_counter() - started
        solved = solve_quiet(stage.model, solver, {"FeasibilityTol": 1e-8}, label=f"{label}/stage2")
        if not solved["incumbent"]:
            # The Stage 1 witnesses satisfy this LP, so an attempt without a solution failed
            # numerically (a false INFEASIBLE on an ill-conditioned model): retry once, carefully.
            out["stage2_first_attempt"] = solve_row(solved, stage.model)
            stage.model.reset()
            solved = solve_quiet(stage.model, solver, {"FeasibilityTol": 1e-8, "NumericFocus": 3},
                                 label=f"{label}/stage2_retry")
        out["stage2"] = solve_row(solved, stage.model)
        if not solved["incumbent"]:
            out["failure"] = "NO_STAGE2_SOLUTION"
            return out
        polynomials, trajectory = extract_stage2(stage, scene, geometry, physics)
    report = validate_continuous(polynomials, scene, geometry, physics, routes, schedules)
    out["validation"] = {key: value for key, value in report.items()
                         if key not in ("extrema", "occupancy_intervals")}
    out["valid"] = bool(report["valid"])
    out["fidelity"] = event_fidelity(report["occupancy_intervals"], schedules)
    if report["valid"]:
        out["cost"] = comparable_cost(trajectory, scene, geometry, physics, validated=True)
    out["realized"] = realized_metrics(trajectory, scene, geometry, physics, routes,
                                       exact_variation=continuous_acceleration_variation(polynomials))
    write_json(Path(folder) / "polynomials.json.gz", polynomials, compress=True)
    return out
