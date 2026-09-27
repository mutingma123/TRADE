"""Vehicle-local trajectory LP with continuous safety certificates."""

from .vehicle_trajectory import add_vehicle_trajectory
from .following_constraints import add_following_constraints

def build_stage2(scenario, geometry, physics, routes, schedules, *, n_segments=12):
    """Build a local-time LP and return a disposable Decomposed lifecycle.Stage.

    ``schedules[vid]`` contains ``entry_s``, ``exit_s`` (downstream clearance),
    and ``passage_segments`` (fixed quintic Bernstein pieces), or a strictly
    positive ``passage_speed_fps`` for an affine passage. Entry velocity and
    acceleration can be supplied as ``entry_speed_fps`` and ``entry_acc_fps2``.
    Polynomial resolution is fixed per vehicle, independent of completion times
    and of any planning horizon; supplied witness switch points are also used.
    """
    import gurobipy as gp
    from gurobipy import GRB
    from Decomposed.solve.lifecycle import constructing_stage

    if n_segments < 1:
        raise ValueError("n_segments must be positive")
    jerk_limit = physics.max_acc_change_fps2 / physics.dt_s
    with constructing_stage("stage2_horizon_free") as stage:
        model, segments, objective = stage.model, {}, []
        fixed_passage_jerk_bound = 0.0
        model.Params.FeasibilityTol = 1e-8
        for vehicle in scenario.vehicles:
            fixed_passage_jerk_bound = add_vehicle_trajectory(model, vehicle, geometry, physics, routes, schedules, n_segments, segments, objective, jerk_limit, fixed_passage_jerk_bound)
        n_gap_intervals = add_following_constraints(model, scenario, segments, schedules, physics)
        model.setObjective(gp.quicksum(objective) + fixed_passage_jerk_bound, GRB.MINIMIZE)
        model.update()
        stage.__dict__.update(segments=segments, schedules=schedules, n_segments=n_segments,
                              n_gap_intervals=n_gap_intervals, jerk_limit_fps3=jerk_limit,
                              fixed_passage_jerk_bound=fixed_passage_jerk_bound)
        return stage
