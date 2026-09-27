"""Shared-cell ordering, exclusion, and exposure constraints."""

import itertools
import gurobipy as gp
from gurobipy import GRB
from .fixed_mode_cuts import _fixed_mode_cuts

def add_conflict_events(model, users, physics, modes, zin, tin, tout):
    order, penalties, clashes = {}, {}, {}
    gap = 2 * 0.2 + physics.dt_s
    for cell, occupants in users.items():
        for first, second in itertools.combinations(occupants, 2):
            a, b = first.vid, second.vid
            pair = a, b
            if pair not in order:
                order[pair] = model.addVar(vtype=GRB.BINARY, name=f'order_{a}_{b}')
                if first.app == second.app and first.lane == second.lane:
                    model.addConstr(order[pair] == int(first.order < second.order))
            y = order[pair]
            model.addGenConstrIndicator(y, 1, tin[b, cell] >= tout[a, cell] + gap)
            model.addGenConstrIndicator(y, 0, tin[a, cell] >= tout[b, cell] + gap)
            if first.app != second.app:
                rho = model.addVar(lb=0.0, name=f'proximity_{a}_{b}_{cell}')
                delta = tin[b, cell] - tin[a, cell]
                model.addGenConstrIndicator(y, 1, rho >= physics.crossing_window_s - delta)
                model.addGenConstrIndicator(y, 0, rho >= physics.crossing_window_s + delta)
                penalties[a, b, cell] = rho
            else:
                rho = None
            _fixed_mode_cuts(model, modes, zin, a, b, cell, y, rho, gap, physics.crossing_window_s, clashes)
    for (a, b), pairs in clashes.items():
        for i in sorted({i for i, _ in pairs}):
            model.addConstr(zin[a][i] + gp.quicksum(zin[b][j] for j in sorted({j for k, j in pairs if k == i}))
                            <= 1, name=f'cell_clash_{a}_{b}_{i}')
    return order, penalties, gap
