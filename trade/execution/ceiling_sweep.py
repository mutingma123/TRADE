"""Solve every exposure ceiling from the loosest to the tightest and realize each schedule."""

import time
import traceback

from gurobipy import GRB

from Decomposed.result.artifacts import write_json
from Decomposed.scan.grid import normalized_caps
from Decomposed.solve.quiet import solve_quiet
from HorizonFree.scheduling.schedule_extract import extract_stage1
from trade.configuration import CAP_TOLERANCE_S
from trade.execution.trajectory_solve import realize
from trade.execution.warm_starts import set_starts, snapshot
from trade.records.run_status import utc_now
from trade.records.solver_records import solve_row


def solve_caps(stage, scene, geometry, physics, solver, ceiling, reward, anchors, solutions,
               root, status, n_caps):
    """Every ceiling from the loosest to the tightest, warm-started from the previous one.

    The previous (looser) solution is a hint; the zero-exposure start, feasible
    under every ceiling, guarantees an incumbent.
    """
    previous, feasible = solutions["fast_tie"], solutions["safe"]
    grid = normalized_caps(anchors["low_hinge_s"], anchors["high_hinge_s"], n_caps)
    for index, position, cap in reversed(grid):
        label = f"caps/cap_{index:02d}"
        status.update(phase=label)
        point = dict(index=index, normalized_position=position, epsilon_s=cap, valid=False)
        started = time.perf_counter()
        try:
            ceiling.RHS = cap + CAP_TOLERANCE_S
            stage.model.setObjective(reward, GRB.MAXIMIZE)
            set_starts(stage.model, [previous, feasible])
            solved = solve_quiet(stage.model, solver, label=f"{label}/stage1")
            point["stage1"] = solve_row(solved, stage.model)
            if solved["incumbent"]:
                previous = snapshot(stage.model)
                schedules, criteria = extract_stage1(stage, scene)
                point["criteria"] = criteria
                point.update(realize(scene, geometry, physics, solver, stage.routes, schedules,
                                     root / label, label))
            else:
                point["failure"] = "NO_STAGE1_INCUMBENT"
        except Exception as error:
            point.update(failure=f"{type(error).__name__}: {error}", traceback=traceback.format_exc())
        point["wall_s"] = time.perf_counter() - started
        write_json(root / label / "point.json", point)
        status.update(caps_done=sorted(status.record["caps_done"] + [index]))
        print(f"{utc_now()} {label}: {point.get('stage1', {}).get('status')} "
              f"valid={point['valid']} {point['wall_s']:.0f} s", flush=True)
