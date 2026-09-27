"""Run one ladder instance: controllers, Stage 1 anchors, and every exposure ceiling."""

import time
import traceback

import gurobipy as gp

from Decomposed.experiment.instance import prepare_instance
from Decomposed.result.artifacts import write_json
from Decomposed.solve.context import recording
from Decomposed.stage1.route_limits import collect_routes
from HorizonFree.scheduling.stage1_model import build_stage1
from trade.configuration import UNBOUNDED_EPSILON_S
from trade.execution.anchor_solves import solve_anchors
from trade.execution.ceiling_sweep import solve_caps
from trade.execution.controller_runs import run_benchmarks
from trade.execution.instance_audit import audit_instance
from trade.records.environment_metadata import environment_record
from trade.records.instance_metadata import instance_record
from trade.records.run_status import Status, utc_now
from trade.records.solver_records import model_counts


def run_instance(args, setting, physics, solver, root):
    wall = time.perf_counter()
    status = Status(root, case=setting.name, seed=args.seed, n_per_lane=args.n_per_lane)
    try:
        scene, geometry = prepare_instance(setting.config(), setting.case(), args.seed)
        routes = collect_routes(scene, geometry)
        write_json(root / "instance.json", instance_record(args, setting, scene, geometry,
                                                           physics, solver, routes))
        write_json(root / "environment.json", environment_record())
        print(f"{utc_now()} {setting.name} seed {args.seed}: {scene.n_vehicles} vehicles", flush=True)
        run_benchmarks(scene, geometry, physics, routes, root / "controllers", status)
        with recording(root, args.time_limit):
            status.update(phase="stage1/build")
            started = time.perf_counter()
            with build_stage1(scene, geometry, physics, UNBOUNDED_EPSILON_S) as stage:
                build = dict(stage1_build_s=time.perf_counter() - started,
                             start_complete=getattr(stage, "start_complete", None),
                             counts=model_counts(stage.model),
                             lane_rules={key: sum(len(rules[key]) for rules in stage.lane_rules.values())
                                         for key in ("forbidden", "min_follow_wait",
                                                     "max_lead_wait", "offsets")})
                ceiling = stage.model.getConstrByName("exposure_ceiling")
                reward = gp.quicksum(stage.reward.values()) * (1.0 / scene.n_vehicles)
                penalty = gp.quicksum(stage.penalties.values())
                anchors, solutions = solve_anchors(stage, scene, solver, reward, penalty, status)
                anchors.update(build)
                write_json(root / "anchors.json", anchors)
                print(f"{utc_now()} anchors valid={anchors['valid']} "
                      f"hinge {anchors.get('low_hinge_s')} to {anchors.get('high_hinge_s')}", flush=True)
                if not anchors["valid"]:
                    status.update(state="ANCHOR_FAILED", phase="anchors", finished_at=utc_now(),
                                  wall_s=time.perf_counter() - wall)
                    return 2
                solve_caps(stage, scene, geometry, physics, solver, ceiling, reward, anchors,
                           solutions, root, status, args.caps)
        audit = audit_instance(root, args.caps)
        status.update(state="COMPLETE" if audit["valid"] else "FAILED", phase="done",
                      failures=audit["failures"], finished_at=utc_now(),
                      wall_s=time.perf_counter() - wall)
        return 0 if audit["valid"] else 3
    except BaseException as error:
        status.update(state="FAILED", error=f"{type(error).__name__}: {error}",
                      traceback=traceback.format_exc(), finished_at=utc_now(),
                      wall_s=time.perf_counter() - wall)
        raise
