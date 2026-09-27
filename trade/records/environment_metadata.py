"""Record source identity and the actual numerical runtime."""

import hashlib
import json
import os
import platform
import socket
import subprocess
import sys
from pathlib import Path

import gurobipy as gp
import numpy as np
import scipy

PACKAGE_ROOT = Path(__file__).resolve().parents[2]


def source_hashes():
    paths = []
    for name in ("Decomposed", "HorizonFree", "trade"):
        paths.extend((PACKAGE_ROOT / name).rglob("*.py"))
        paths.extend((PACKAGE_ROOT / name).rglob("*.csv"))
    return {str(p.relative_to(PACKAGE_ROOT)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in sorted(paths)}


def source_fingerprint():
    return hashlib.sha256(json.dumps(source_hashes(), sort_keys=True).encode()).hexdigest()


def environment_record():
    def git(*arguments):
        if not (PACKAGE_ROOT / ".git").exists():
            return None
        try:
            result = subprocess.run(["git", *arguments], cwd=PACKAGE_ROOT,
                                    capture_output=True, text=True, timeout=30)
            return result.stdout.strip() if result.returncode == 0 else None
        except (OSError, subprocess.SubprocessError):
            return None

    return dict(python=sys.version, executable=sys.executable, platform=platform.platform(),
                machine=platform.machine(), host=socket.gethostname(), cpu_count=os.cpu_count(),
                gurobi=".".join(map(str, gp.gurobi.version())), numpy=np.__version__,
                scipy=scipy.__version__, git_head=git("rev-parse", "HEAD"),
                git_source_status=git("status", "--porcelain", "--", "HorizonFree", "Decomposed", "trade"),
                source_sha256=source_hashes())
