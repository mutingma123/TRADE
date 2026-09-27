"""Study handle: the physical configuration, the solver settings, and the geometry folder."""

from __future__ import annotations

from dataclasses import dataclass, field

from .physics import PhysicsConfig
from .solver import SolverConfig


@dataclass(frozen=True)
class StudyConfig:
    geometry_data_dir: str = "data/geometry"
    physics: PhysicsConfig = field(default_factory=PhysicsConfig)
    solver: SolverConfig = field(default_factory=SolverConfig)
