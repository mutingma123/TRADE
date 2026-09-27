"""Stage 1 anchors: the fastest schedule, its least-exposure tie, and the zero-exposure end."""

from gurobipy import GRB

from Decomposed.solve.quiet import solve_quiet
from trade.configuration import SPEED_TIE_FPS
from trade.execution.warm_starts import defined_starts, set_starts, snapshot
from trade.records.solver_records import solve_row


def stage_hinge(stage):
    """The event hinge of the incumbent, from its cell entry times."""
    window = stage.crossing_window_s
    return sum(max(0.0, window - abs(stage.tin[a, cell].X - stage.tin[b, cell].X))
               for a, b, cell in stage.penalties)


def stage_speed(stage, scene):
    """The exact mean capped approach-speed criterion of the incumbent."""
    return sum(min(stage.caps[v.vid], stage.distances[v.vid] / (stage.entry[v.vid].X - v.release_s))
               for v in scene.vehicles) / scene.n_vehicles


def solve_anchors(stage, scene, solver, reward, penalty, status):
    """fast: the largest speed with no ceiling. fast_tie: the least exposure at that speed.

    The maximum-speed optimum is degenerate in exposure (a vehicle already at
    its speed cap can move without losing reward), so fast_tie fixes the
    nondominated top of the ceiling grid. The bottom is zero exposure, which the
    constructive start certifies; safe is solved only when that start is incomplete.
    """
    model = stage.model
    records, solutions = {}, {"constructive": defined_starts(model)}

    def optimize(name, objective, sense, starts):
        status.update(phase=f"anchors/{name}")
        model.setObjective(objective, sense)
        set_starts(model, starts)
        solved = solve_quiet(model, solver, label=f"anchors/{name}")
        records[name] = solve_row(solved, model)
        if solved["incumbent"]:
            solutions[name] = snapshot(model)
            records[name].update(speed_fps=stage_speed(stage, scene), hinge_s=stage_hinge(stage))
        return solved["incumbent"]

    result = dict(valid=False, solves=records)
    if not optimize("fast", reward, GRB.MAXIMIZE, [solutions["constructive"]]):
        return dict(result, failed_anchor="fast"), solutions
    fast_reward = model.ObjVal
    floor = model.addConstr(reward >= fast_reward - SPEED_TIE_FPS, name="speed_floor")
    try:
        if not optimize("fast_tie", penalty, GRB.MINIMIZE, [solutions["fast"]]):
            return dict(result, failed_anchor="fast_tie"), solutions
    finally:
        model.remove(floor)
        model.update()
    high = records["fast_tie"]["hinge_s"]
    if getattr(stage, "start_complete", False):
        low, source = 0.0, "constructive zero-exposure start"
        solutions["safe"] = solutions["constructive"]
    else:
        if not optimize("safe", penalty, GRB.MINIMIZE, [solutions["fast_tie"]]):
            return dict(result, failed_anchor="safe"), solutions
        low, source = records["safe"]["hinge_s"], "safe anchor solve"
    if low > high + 1e-9:
        return dict(result, failed_anchor="anchor_order", low_hinge_s=low, high_hinge_s=high), solutions
    return dict(result, valid=True, low_hinge_s=low, high_hinge_s=high, low_source=source,
                fast_reward_fps=fast_reward), solutions
