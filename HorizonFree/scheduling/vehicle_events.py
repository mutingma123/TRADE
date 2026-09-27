"""Vehicle mode, cell event, and approach reward variables."""

import gurobipy as gp
from gurobipy import GRB
from Decomposed.stage1.route_limits import reachable_entry_speed, speed_cap
from .speed_tangents import tangent_speeds, tangent_error

def add_vehicle_events(model, scenario, physics, routes, modes):
    entry, finish, zin, reward, tangents, errors = {}, {}, {}, {}, {}, {}
    tin, tout, waits = {}, {}, {}
    caps, distances = {}, {}
    for v in scenario.vehicles:
        vid = v.vid
        cap = reachable_entry_speed(v, physics, speed_cap(v, physics))[1]
        distance = routes[vid][0].front_entry_ft - v.ini_dis_ft
        caps[vid], distances[vid] = cap, distance
        entry[vid] = model.addVar(lb=v.release_s, name=f'entry_{vid}')
        finish[vid] = model.addVar(lb=v.release_s, name=f'finish_{vid}')
        waits[vid] = model.addVar(lb=0.0, name=f'wait_{vid}')
        reward[vid] = model.addVar(lb=0.0, ub=cap, name=f'speed_proxy_{vid}')
        zin[vid] = [model.addVar(vtype=GRB.BINARY, name=f'mode_{vid}_{m["name"]}') for m in modes[vid]]
        model.addConstr(gp.quicksum(zin[vid]) == 1, name=f'one_mode_{vid}')
        model.addConstr(entry[vid] == waits[vid] + gp.quicksum(
            z * m['entry_s'] for z, m in zip(zin[vid], modes[vid])))
        model.addConstr(finish[vid] == entry[vid] + gp.quicksum(
            z * m['duration_s'] for z, m in zip(zin[vid], modes[vid])))
        # Valid linking cut: a waiting mode's reward never exceeds its zero-wait value.
        model.addConstr(reward[vid] <= gp.quicksum(
            z * min(cap, distance / (m['entry_s'] - v.release_s)) for z, m in zip(zin[vid], modes[vid])),
            name=f'reward_link_{vid}')
        for k, mode in enumerate(modes[vid]):
            z = zin[vid][k]
            if mode['name'] != 'stop':
                model.addGenConstrIndicator(z, 1, waits[vid] == 0)
                exact = min(cap, distance / (mode['entry_s'] - v.release_s))
                model.addGenConstrIndicator(z, 1, reward[vid] == exact)
            else:
                knots = tangent_speeds(min(cap, distance / (mode['entry_s'] - v.release_s)))
                selectors = [model.addVar(vtype=GRB.BINARY, name=f'tangent_{vid}_{j}') for j in range(len(knots))]
                model.addConstr(gp.quicksum(selectors) == z)
                for select, speed in zip(selectors, knots):
                    line = 2 * speed - speed ** 2 / distance * (entry[vid] - v.release_s)
                    model.addGenConstrIndicator(select, 1, reward[vid] <= line)
                tangents[vid] = knots, selectors
                errors[vid] = tangent_error(knots)
        for th in routes[vid]:
            key = vid, th.cell
            tin[key] = model.addVar(lb=v.release_s, name=f'in_{vid}_{th.cell}')
            tout[key] = model.addVar(lb=v.release_s, name=f'out_{vid}_{th.cell}')
            model.addConstr(tin[key] == entry[vid] + gp.quicksum(
                z * m['entries'][th.cell] for z, m in zip(zin[vid], modes[vid])))
            model.addConstr(tout[key] == entry[vid] + gp.quicksum(
                z * m['exits'][th.cell] for z, m in zip(zin[vid], modes[vid])))
    return entry, finish, zin, reward, tangents, errors, tin, tout, waits, caps, distances
