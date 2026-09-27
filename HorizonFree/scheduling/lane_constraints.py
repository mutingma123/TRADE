"""FIFO, passage headways, and witness compatibility rows."""

import gurobipy as gp
from gurobipy import GRB
from Decomposed.geometry.following import following_pairs
from Decomposed.scenario.leaders import leader_of
from ..motion.lane_rules import pair_rules

def add_lane_constraints(model, scenario, physics, routes, modes, zin, entry, waits):
    for follower in scenario.vehicles:
        leader = leader_of(follower, scenario.vehicles)
        if leader is None:
            continue
        a, b = leader.vid, follower.vid
        model.addConstr(entry[b] >= entry[a], name=f'fifo_{a}_{b}')
        if leader.move == follower.move:
            distance = physics.vehicle_length_ft + physics.safe_buffer_ft
            offset = routes[b][0].front_entry_ft - routes[a][0].front_entry_ft
            headway = gp.quicksum(z * (distance + offset + m['speed_deficit_ft']) / m['cap_fps']
                                 for z, m in zip(zin[a], modes[a]))
            model.addConstr(entry[b] >= entry[a] + headway, name=f'following_passage_{a}_{b}')
    lane_rules, both_stop = {}, {}
    for leader, follower in following_pairs(scenario.vehicles):
        a, b = leader.vid, follower.vid
        rules = pair_rules(leader, follower, modes[a], modes[b], physics)
        lane_rules[a, b] = rules
        rules['forbidden'] = set(rules['forbidden'])
        for i, j in sorted(rules['forbidden']):
            model.addConstr(zin[a][i] + zin[b][j] <= 1, name=f'lane_forbid_{a}_{b}_{i}_{j}')
        for (i, j), wait in rules['min_follow_wait'].items():
            model.addConstr(waits[b] >= wait * (zin[a][i] + zin[b][j] - 1),
                            name=f'lane_follow_wait_{a}_{b}_{i}_{j}')
        for (i, j), wait in rules['max_lead_wait'].items():
            model.addGenConstrIndicator(zin[b][j], 1, waits[a] <= wait)
        for (i, j), offset in rules['offsets'].items():
            y = model.addVar(vtype=GRB.BINARY, name=f'both_wait_{a}_{b}')
            model.addConstr(y >= zin[a][i] + zin[b][j] - 1)
            model.addGenConstrIndicator(y, 1, waits[b] - waits[a] >= offset)
            both_stop[a, b] = y
    return lane_rules, both_stop
