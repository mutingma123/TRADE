"""Horizon-free event MILP using attainable physical motion witnesses."""

import math
import gurobipy as gp
from gurobipy import GRB
from Decomposed.solve.lifecycle import constructing_stage
from Decomposed.stage1.route_limits import collect_routes, cell_users
from ..motion.mode_catalogue import build_modes
from .vehicle_events import add_vehicle_events
from .conflict_events import add_conflict_events
from .lane_constraints import add_lane_constraints
from .constructive_start import _start

def build_stage1(scenario, geometry, physics, epsilon):
    if not math.isfinite(epsilon) or epsilon < 0:
        raise ValueError('The proximity ceiling must be finite and nonnegative')
    routes = collect_routes(scenario, geometry)
    modes = {v.vid: build_modes(v, geometry, physics, routes[v.vid]) for v in scenario.vehicles}
    users = cell_users(scenario, routes)
    with constructing_stage('stage1_horizon_free') as stage:
        model = stage.model
        (entry, finish, zin, reward, tangents, errors, tin, tout, waits, caps, distances) = add_vehicle_events(model, scenario, physics, routes, modes)
        order, penalties, gap = add_conflict_events(model, users, physics, modes, zin, tin, tout)
        lane_rules, both_stop = add_lane_constraints(model, scenario, physics, routes, modes, zin, entry, waits)
        model.addConstr(gp.quicksum(penalties.values()) <= epsilon + 1e-4, name='exposure_ceiling')
        objective = gp.quicksum(reward.values()) / scenario.n_vehicles
        model.setObjective(objective, GRB.MAXIMIZE)
        model.update()
        stage.__dict__.update(routes=routes, modes=modes, entry=entry, finish=finish, zin=zin,
                              reward=reward, tangents=tangents, tin=tin, tout=tout, waits=waits,
                              order=order, penalties=penalties, caps=caps, distances=distances,
                              lane_rules=lane_rules, both_stop=both_stop,
                              reward_error_bound_fps=sum(errors.values()) / scenario.n_vehicles,
                              epsilon=epsilon, gap_s=gap, crossing_window_s=physics.crossing_window_s)
        _start(stage, scenario, physics)
        return stage
