"""Atomic JSON records shared by solver archives and experiment runs."""

import gzip
import json
import math
import os
from pathlib import Path

import numpy as np
from gurobipy import GRB


def plain(value):
    """A JSON-safe copy: non-finite floats become null and numpy scalars become Python."""
    if isinstance(value, dict):
        return {str(key): plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [plain(item) for item in value]
    if isinstance(value, np.generic):
        value = value.item()
    if isinstance(value, float) and (not math.isfinite(value) or abs(value) >= GRB.INFINITY):
        return None
    return value


def write_json(path, record, compress=False):
    """Publish a JSON record atomically, gzip-compressed when asked."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    text = json.dumps(plain(record), indent=None if compress else 1, allow_nan=False)
    temporary = path.with_name(path.name + ".tmp")
    if compress:
        with gzip.open(temporary, "wt") as handle:
            handle.write(text)
    else:
        temporary.write_text(text + "\n")
    os.replace(temporary, path)


def read_json(path):
    """A JSON record, gzip-compressed or not, or None when it does not exist."""
    path = Path(path)
    if not path.exists():
        return None
    if path.suffix == ".gz":
        with gzip.open(path, "rt") as handle:
            return json.load(handle)
    return json.loads(path.read_text())
