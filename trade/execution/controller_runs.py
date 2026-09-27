"""Run, validate, and price the three rule-based controllers on one instance."""

import time
import traceback
from pathlib import Path

from Decomposed.benchmarks.rulebased import run_rule_controller
from Decomposed.config.benchmark import BenchmarkConfig
from Decomposed.result.artifacts import write_json
from Decomposed.result.trajectory.adapters import from_controller_result
from Decomposed.validation.report import validate_trajectory
from HorizonFree.metrics import comparable_cost
from trade.configuration import RULES
from trade.execution.realized_measures import realized_metrics
from trade.records.run_status import utc_now
from trade.records.trajectory_csv import write_trajectory_gz


def run_benchmarks(scene, geometry, physics, routes, folder, status):
    """The three rule-based controllers on the same instance, priced like the frontier."""
    for name in RULES:
        status.update(phase=f"benchmarks/{name}")
        started = time.perf_counter()
        row = dict(controller=name)
        try:
            result = run_rule_controller(name, scene, geometry, physics, BenchmarkConfig())
            trajectory = from_controller_result(result, scene, geometry, physics)
            report = validate_trajectory(trajectory, scene, geometry, physics)
            row.update(status=result.status, failure=result.failure, valid=bool(report["valid"]),
                       failures=report.get("failures"),
                       horizon_extension_s=report.get("horizon_extension_s"),
                       provenance=result.provenance.to_dict() if result.provenance else None)
            if report["valid"]:
                row["cost"] = comparable_cost(trajectory, scene, geometry, physics, validated=True)
            row["realized"] = realized_metrics(trajectory, scene, geometry, physics, routes)
            write_trajectory_gz(Path(folder) / name / "trajectory.csv.gz", trajectory)
        except Exception as error:
            row.update(status="ERROR", valid=False, error=f"{type(error).__name__}: {error}",
                       traceback=traceback.format_exc())
        row["elapsed_s"] = time.perf_counter() - started
        write_json(Path(folder) / name / "result.json", row)
        print(f"{utc_now()} benchmark {name}: valid={row.get('valid')} {row['elapsed_s']:.1f} s", flush=True)
