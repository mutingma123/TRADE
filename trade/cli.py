"""Command-line entry point for execution and artifact checks."""

import json

from Decomposed.result.artifacts import read_json
from trade.arguments import parse_arguments
from trade.execution.instance_audit import audit_instance
from trade.execution.instance_request import run_one
from trade.ladder import run_ladder


def check_runs(root):
    paths = [root] if (root / "instance.json").exists() else sorted(
        path for path in root.glob("v*_*_50/seed_*") if path.is_dir() and "." not in path.name)
    valid = bool(paths)
    for path in paths:
        report = audit_instance(path)
        status = read_json(path / "status.json") or {}
        report["root"] = str(path)
        report["valid"] &= status.get("state") == "COMPLETE"
        valid &= report["valid"]
        print(json.dumps(report))
    if not paths:
        print(f"No experiment instances found under {root}")
    return 0 if valid else 1


def main(argv=None):
    args = parse_arguments(argv)
    try:
        if args.command == "instance":
            return run_one(args)
        if args.command == "ladder":
            return run_ladder(args)
        return check_runs(args.root)
    except (OSError, ValueError, RuntimeError) as error:
        print(f"TRADE: {error}")
        return 1
