"""Archive every optimization: its model, warm start, log, solution, and record."""

from pathlib import Path
from gurobipy import GRB

from ..result.artifacts import write_json
from .context import CURRENT
from .progress import Progress

# Gurobi 13.0.3 aborts the whole process (SIGTRAP, buffer overflow) when
# Model.write receives a path of 244 or more characters.
GUROBI_PATH_LIMIT = 243


def check_path(path):
    if len(str(path)) > GUROBI_PATH_LIMIT:
        raise ValueError(f"Gurobi cannot write paths over {GUROBI_PATH_LIMIT} characters; "
                         f"use a shorter results root: {path}")
    return str(path)


def captured_solve(model, config, overrides, label, optimize):
    """One recorded attempt that stops at the target gap or the solve cap, whichever comes first."""
    context = CURRENT.get()
    if context is None:
        return optimize(model, config, overrides)
    relative = Path(label)
    if relative.is_absolute() or ".." in relative.parts:
        raise ValueError("Solve labels must stay within the experiment directory")
    folder = context.root / relative / "solve_01"
    check_path(folder / "solution.sol.gz")  # the longest name written here, checked before solving
    folder.mkdir(parents=True, exist_ok=False)
    context.progress(phase=label, state="SOLVING")
    parameters = {**(overrides or {}), "Method": 1, "ConcurrentMIP": 1,
                  "LogToConsole": 0, "OutputFlag": 1,
                  "LogFile": str(folder / "solver.log"), "NodefileDir": str(folder)}
    budget = parameters.get("TimeLimit", GRB.INFINITY if config.unlimited else config.time_limit_s)
    capped = context.solve_cap_s <= budget
    parameters["TimeLimit"] = min(budget, context.solve_cap_s)
    model.update()
    model.write(str(folder / "model.mps.gz"))
    starts = [[var.VarName, var.Start] for var in model.getVars()
              if -GRB.INFINITY < var.Start < GRB.INFINITY]
    write_json(folder / "warm_start.json", starts)
    progress = Progress(folder / "progress.json")
    try:
        result = optimize(model, config, parameters, progress)
        if progress.error:
            raise progress.error
        if result["incumbent"]:
            model.write(str(folder / "solution.sol.gz"))
        result["artifacts"] = str(folder.relative_to(context.root))
        write_json(folder / "solve.json", result)
    except BaseException as error:
        write_json(folder / "error.json", {"error": str(error), "type": type(error).__name__})
        raise
    finally:
        model.setParam("LogFile", "")
        model.setParam("OutputFlag", 0)
    if (capped and result["status"] == "TIME_LIMIT" and result["incumbent"]
            and not result["quality"]["accepted"]):
        result["quality"] = {**result["quality"], "label": "CAPPED", "capped": True,
                             "cap_s": context.solve_cap_s, "mip_gap": result.get("mip_gap")}
    return result
