"""Check that every requested ceiling and controller produced valid artifacts."""

from pathlib import Path

from Decomposed.result.artifacts import read_json
from trade.configuration import RULES


def audit_instance(root, n_caps=None):
    root = Path(root)
    failures = []

    def record(relative):
        try:
            value = read_json(root / relative)
            if not isinstance(value, dict):
                raise ValueError("missing or malformed record")
            return value
        except (OSError, ValueError) as error:
            failures.append(f"{relative}: {error}")
            return {}

    instance = record("instance.json")
    n_caps = n_caps if n_caps is not None else instance.get("n_caps", 0)
    if not isinstance(n_caps, int) or n_caps < 2:
        failures.append("missing or invalid ceiling count")
        n_caps = 0
    if not record("anchors.json").get("valid"):
        failures.append("invalid anchors")
    for label in [f"caps/cap_{i:02d}" for i in range(n_caps)]:
        point = record(f"{label}/point.json")
        if not point.get("valid") or not point.get("cost"):
            failures.append(f"{label}: missing validated cost")
        for stage in ("stage1", "stage2"):
            if not point.get(stage, {}).get("incumbent"):
                failures.append(f"{label}: missing {stage} incumbent")
        polynomial = record(f"{label}/polynomials.json.gz")
        if len(polynomial.get("vehicles", {})) != instance.get("n_vehicles"):
            failures.append(f"{label}: incomplete polynomial trajectories")
    for name in RULES:
        label = f"controllers/{name}"
        result = record(f"{label}/result.json")
        if not result.get("valid") or not result.get("cost"):
            failures.append(f"{label}: missing validated cost")
        path = root / label / "trajectory.csv.gz"
        if not path.is_file() or path.stat().st_size == 0:
            failures.append(f"{label}: missing sampled trajectory")
    return dict(valid=not failures, failures=failures, n_caps=n_caps,
                controllers=len(RULES))
