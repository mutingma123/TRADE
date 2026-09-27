"""Incumbent snapshots and MIP starts carried between Stage 1 solves."""

import gurobipy as gp
from gurobipy import GRB


def snapshot(model):
    """Every variable value of the incumbent, by name, or None."""
    if model.SolCount <= 0:
        return None
    return {var.VarName: var.X for var in model.getVars()}


def defined_starts(model):
    """The MIP start currently installed, by name."""
    return {var.VarName: var.Start for var in model.getVars() if abs(var.Start) < GRB.INFINITY}


def set_starts(model, solutions):
    """Install one MIP start per solution; by convention the last one is feasible."""
    solutions = [solution for solution in solutions if solution]
    if not solutions:
        return 0
    variables = model.getVars()
    try:
        model.Params.StartNumber = 0
        model.NumStart = len(solutions)
        model.update()
        for number, solution in enumerate(solutions):
            model.Params.StartNumber = number
            for var in variables:
                var.Start = solution.get(var.VarName, GRB.UNDEFINED)
        model.update()
        return len(solutions)
    except (gp.GurobiError, AttributeError):
        try:
            model.NumStart = 1
            model.update()
            model.Params.StartNumber = 0
        except (gp.GurobiError, AttributeError):
            pass
        for var in variables:
            var.Start = solutions[-1].get(var.VarName, GRB.UNDEFINED)
        model.update()
        return 1
