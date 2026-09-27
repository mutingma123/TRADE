"""Solve with explicit bounds, gap certificates, reproducibility, and optional archives."""

import time
from gurobipy import GRB

from .parameters import apply_solver_parameters
from .capture import captured_solve
from .quality import certificate, finite

STATUS_NAMES = {getattr(GRB, name): name for name in (
    "OPTIMAL", "TIME_LIMIT", "WORK_LIMIT", "INFEASIBLE", "INF_OR_UNBD", "UNBOUNDED",
    "SUBOPTIMAL", "INTERRUPTED", "NUMERIC", "MEM_LIMIT", "NODE_LIMIT", "SOLUTION_LIMIT")}


def _optimize(model, config, overrides=None, callback=None):
    parameters = {"Method": 1, "ConcurrentMIP": 1, **(overrides or {})}
    model.setParam("LogToConsole", 0)
    applied = apply_solver_parameters(model, config, parameters)
    model.setParam("OutputFlag", parameters.get("OutputFlag", 0))
    started = time.perf_counter()
    model.optimize(callback) if callback else model.optimize()
    incumbent = model.SolCount > 0
    objective = finite(model.ObjVal) if incumbent else None
    bound = finite(getattr(model, "ObjBound", None))
    if not model.IsMIP and model.Status == GRB.OPTIMAL:
        bound = objective
    gap = finite(getattr(model, "MIPGap", None)) if incumbent and model.IsMIP else None
    result = dict(status=STATUS_NAMES.get(model.Status, str(model.Status)),
                  elapsed_s=time.perf_counter() - started, incumbent=incumbent,
                  obj=objective, objective_bound=bound, mip_gap=gap,
                  absolute_gap=None if objective is None or bound is None else abs(objective - bound),
                  work=finite(getattr(model, "Work", None)), parameters=applied,
                  model_fingerprint=getattr(model, "Fingerprint", None),
                  variables=model.NumVars, binaries=model.NumBinVars,
                  constraints=model.NumConstrs)
    result["quality"] = certificate(result, config)
    return result


def solve_quiet(model, config, overrides=None, *, label="solve"):
    return captured_solve(model, config, overrides, label, _optimize)
