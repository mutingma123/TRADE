"""The instance.json record: settings, scenario identity, and every vehicle."""

from Decomposed.config.benchmark import BenchmarkConfig


def instance_record(args, setting, scene, geometry, physics, solver, routes):
    vehicles = [dict(vehicle.to_dict(), boundary_ft=routes[vehicle.vid][0].front_entry_ft,
                     target_ft=geometry.clearance_target_ft(vehicle.app, vehicle.lane, vehicle.move))
                for vehicle in scene.vehicles]
    case = setting.case()
    return dict(case=setting.name, n_per_lane=args.n_per_lane, n_vehicles=scene.n_vehicles,
                seed=args.seed, arrival=case.arrival, turn_fraction=case.turn_fraction,
                scenario_hash=scene.scenario_hash, geometry_hash=geometry.geometry_hash,
                n_caps=args.caps, time_limit_s=args.time_limit, mip_gap=args.mip_gap,
                physics=physics.to_dict(), solver=solver.to_dict(),
                benchmark=BenchmarkConfig().to_dict(), vehicles=vehicles,
                horizon_note="physics.horizon_s is the design value; HorizonFree never reads it")
