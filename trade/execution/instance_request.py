"""Bind resumed output to the exact settings and computational source files."""

from dataclasses import replace
from datetime import datetime, timezone
from uuid import uuid4

from Decomposed.experiment.design import setting_named
from Decomposed.result.artifacts import read_json, write_json
from trade.execution.instance_audit import audit_instance
from trade.execution.instance_lock import instance_lock
from trade.execution.instance_run import run_instance
from trade.records.environment_metadata import source_fingerprint


def run_one(args):
    setting = setting_named(f"v{8 * args.n_per_lane}_{args.arrival}_50")
    if not 1 <= args.seed <= setting.n_seeds:
        raise ValueError(f"Seed must be in 1..{setting.n_seeds} for {setting.name}")
    config = setting.config()
    solver = replace(config.solver, time_limit_s=args.time_limit, mip_gap=args.mip_gap,
                     threads_per_solve=args.threads, memory_gb_per_solve=args.memory_gb)
    request = dict(schema=1, case=setting.name, seed=args.seed, caps=args.caps,
                   physics=config.physics.to_dict(), solver=solver.to_dict(),
                   source_fingerprint=source_fingerprint())
    root = args.root.resolve() / setting.name / f"seed_{args.seed:04d}"
    with instance_lock(root.with_name(root.name + ".lock")):
        previous = read_json(root / "status.json")
        if previous and previous.get("state") == "COMPLETE":
            if read_json(root / "request.json") != request:
                raise ValueError(f"Completed output uses different settings or sources: {root}; use a new root")
            if audit_instance(root, args.caps)["valid"]:
                print(f"{setting.name} seed {args.seed}: already complete", flush=True)
                return 0
        if root.exists():
            stamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
            aside = root.with_name(f"{root.name}.incomplete-{stamp}-{uuid4().hex[:8]}")
            root.rename(aside)
        root.mkdir(parents=True)
        write_json(root / "request.json", request)
        return run_instance(args, setting, config.physics, solver, root)
