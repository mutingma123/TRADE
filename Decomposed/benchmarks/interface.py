"""The common controller result schema and its provenance record.

Every controller takes the same scenario, geometry, and physics and returns
the same result shape, so a comparison never depends on how a schedule was
produced. Provenance names the algorithm, its source, the adaptation made
here, and the parameter version, which is what makes the comparison auditable.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Optional


@dataclass(frozen=True)
class ControllerProvenance:
    controller: str
    algorithm: str
    source: str
    adaptation: str
    parameter_version: str

    def to_dict(self) -> dict:
        return asdict(self)


@dataclass
class ControllerResult:
    controller: str
    status: str
    states: list = field(default_factory=list)
    decisions: list = field(default_factory=list)
    provenance: Optional[ControllerProvenance] = None
    failure: Optional[str] = None
