"""Solve rows kept with each point and the size of a built model."""

import gurobipy as gp


def attribute(model, name):
    try:
        return getattr(model, name)
    except (AttributeError, gp.GurobiError):
        return None


def solve_row(solved, model):
    """The solve record kept with each point: status, time, bound, gap and size."""
    keep = ("status", "elapsed_s", "incumbent", "obj", "objective_bound", "mip_gap",
            "absolute_gap", "work", "variables", "binaries", "constraints", "artifacts")
    row = {key: solved.get(key) for key in keep}
    row.update(quality=(solved.get("quality") or {}).get("label"),
               nodes=attribute(model, "NodeCount"), iterations=attribute(model, "IterCount"))
    return row


def model_counts(model):
    model.update()
    return dict(variables=model.NumVars, binaries=model.NumBinVars,
                linear_constraints=model.NumConstrs, general_constraints=model.NumGenConstrs,
                nonzeros=model.NumNZs)
