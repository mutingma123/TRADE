"""Valid cuts for fixed nonstopping cell events."""

import gurobipy as gp

def _fixed_mode_cuts(model, modes, zin, a, b, cell, y, rho, gap, window, clashes):
    """Valid cuts from nonstopping mode pairs, whose cell events are constants.

    For each nonstopping mode i of ``a``, at most one mode of ``b`` is chosen,
    so aggregating over j keeps every inequality valid. Pairs that overlap in
    the cell under either order are collected in ``clashes``.
    """
    for i, first in enumerate(modes[a]):
        if first['wait_start_s'] is not None:
            continue
        a_in, a_out = first['entry_s'] + first['entries'][cell], first['entry_s'] + first['exits'][cell]
        a_first, b_first, hinge = [], [], []
        for j, second in enumerate(modes[b]):
            if second['wait_start_s'] is not None:
                continue
            b_in, b_out = second['entry_s'] + second['entries'][cell], second['entry_s'] + second['exits'][cell]
            if b_in >= a_out + gap:
                a_first.append(j)
            elif a_in >= b_out + gap:
                b_first.append(j)
            else:
                clashes.setdefault((a, b), set()).add((i, j))
                continue
            value = max(0.0, window - abs(b_in - a_in))
            if value > 0.0:
                hinge.append((j, value))
        if a_first:
            model.addConstr(y >= zin[a][i] + gp.quicksum(zin[b][j] for j in a_first) - 1)
        if b_first:
            model.addConstr(y <= 2 - zin[a][i] - gp.quicksum(zin[b][j] for j in b_first))
        if rho is not None and hinge:
            model.addConstr(rho >= gp.quicksum(value * zin[b][j] for j, value in hinge) - window * (1 - zin[a][i]))
