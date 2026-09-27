"""Completion: every vehicle meets the same declared clearance target.

A schedule is eligible for performance comparison only when all of its
vehicles clear, because a partially cleared batch and a fully cleared batch are
not comparable on delay or throughput.
"""

from __future__ import annotations


def check_completion(clearances, scenario) -> list:
    failures = []
    covered = {record.vid for record in clearances}
    missing = {v.vid for v in scenario.vehicles} - covered
    if missing:
        failures.append(f"no clearance record for vehicles {sorted(missing)}")
    for record in clearances:
        if not record.available:
            failures.append(f"vehicle {record.vid} never reaches its clearance target")
    return failures
