"""SolverConfig: termination, tolerances, resources, and reproducibility.

`time_limit_s=None` is the default and means unlimited solving. The solver
component translates it to solver infinity; nothing in the runtime converts it
to a numeric zero, which would terminate immediately. A finite budget applies
only when an experiment explicitly overrides this field.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Optional

from .validators import fraction, nonnegative, positive


@dataclass(frozen=True)
class SolverConfig:
    time_limit_s: Optional[float] = None
    mip_gap: float = 0.01
    mip_gap_abs: float = 1e-10
    nodefile_start_gb: Optional[float] = None
    integrality_tol: float = 1e-6
    feasibility_tol: float = 1e-6
    threads_per_solve: int = 2
    memory_gb_per_solve: float = 12.0
    solver_seed: int = 9
    mip_focus: int = 1
    heuristics: float = 0.5

    def __post_init__(self) -> None:
        if self.time_limit_s == 0:
            object.__setattr__(self, "time_limit_s", None)
        if self.time_limit_s is not None:
            positive("time_limit_s", self.time_limit_s)
        fraction("mip_gap", self.mip_gap)
        nonnegative("mip_gap_abs", self.mip_gap_abs)
        if self.nodefile_start_gb is not None:
            nonnegative("nodefile_start_gb", self.nodefile_start_gb)
        fraction("heuristics", self.heuristics)
        positive("integrality_tol", self.integrality_tol)
        positive("feasibility_tol", self.feasibility_tol)
        positive("memory_gb_per_solve", self.memory_gb_per_solve)
        if self.threads_per_solve < 1:
            raise ValueError("threads_per_solve must be at least one")
        if self.mip_focus not in (0, 1, 2, 3):
            raise ValueError("mip_focus must be a documented solver focus level")

    @property
    def unlimited(self) -> bool:
        return self.time_limit_s is None

    def to_dict(self) -> dict:
        return asdict(self)
