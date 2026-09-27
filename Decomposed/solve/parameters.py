"""Translate SolverConfig into solver parameters.

Unlimited solving is the configured default and becomes the solver's infinity
setting. It is never written as a numeric zero, which the solver would read as
an immediate termination. A finite ceiling is applied only when the study
explicitly overrides the field, and the override is recorded with the solve.
"""

from __future__ import annotations

from gurobipy import GRB


def solver_parameters(config) -> dict:
    """The full parameter set one solve applies, unlimited by default."""
    parameters = {
        "TimeLimit": GRB.INFINITY if config.unlimited else config.time_limit_s,
        "MIPGap": config.mip_gap,
        "MIPGapAbs": config.mip_gap_abs,
        "IntFeasTol": config.integrality_tol,
        "FeasibilityTol": config.feasibility_tol,
        "Threads": config.threads_per_solve,
        "Seed": config.solver_seed,
        "MIPFocus": config.mip_focus,
        "Heuristics": config.heuristics,
        "SoftMemLimit": config.memory_gb_per_solve,
    }
    if config.nodefile_start_gb is not None:
        parameters["NodefileStart"] = config.nodefile_start_gb
    return parameters


def apply_solver_parameters(model, config, overrides: dict = None) -> dict:
    """Apply the parameters and return exactly what was set, for the record."""
    applied = solver_parameters(config)
    applied.update(overrides or {})
    for name, value in applied.items():
        model.setParam(name, value)
    return applied
