"""Per-process solve archives and the solve time cap, scoped to one instance."""

from contextlib import contextmanager
from contextvars import ContextVar
from dataclasses import dataclass
from pathlib import Path

from ..result.artifacts import write_json

CURRENT = ContextVar("decomposed_run", default=None)


@dataclass
class RunContext:
    root: Path
    solve_cap_s: float  # solving seconds allowed to each optimization

    def progress(self, **values):
        write_json(self.root / "progress.json", values)


@contextmanager
def recording(root, solve_cap_s):
    if not solve_cap_s > 0:
        raise ValueError("solve_cap_s must be positive")
    context = RunContext(Path(root).resolve(), solve_cap_s)
    context.root.mkdir(parents=True, exist_ok=True)
    token = CURRENT.set(context)
    try:
        yield context
    finally:
        CURRENT.reset(token)
