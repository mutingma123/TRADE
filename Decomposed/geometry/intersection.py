"""Geometry: the versioned intersection every component shares.

Four approaches, eight lanes, and sixteen conflict cells. The geometry hash
covers the cell table, the vehicle length, and the clearance span, so any
change to the partition or to full body occupancy forces occupancy intervals
to be recomputed rather than reused.
"""

from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
from pathlib import Path

from ..config.hashing import fingerprint
from .cells import legal_moves, read_cell_conditions
from .routes import build_route
from .thresholds import thresholds_for

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
APPROACHES = (1, 2, 3, 4)
LANES = (1, 2)


@dataclass(frozen=True)
class Geometry:
    conditions: tuple
    n_cells: int
    vehicle_length_ft: float
    border_ft: float
    after_length_ft: float
    version: str

    @lru_cache(maxsize=None)
    def route(self, app: int, lane: int, move: int):
        """Ordered occupied cells for one legal movement."""
        return build_route(self.conditions, app, lane, move)

    @lru_cache(maxsize=None)
    def thresholds(self, app: int, lane: int, move: int) -> tuple:
        """Front entry and rear exit distances for each cell of the route."""
        return thresholds_for(self.route(app, lane, move), self.vehicle_length_ft)

    def exit_ft(self, app: int, lane: int, move: int) -> float:
        return self.route(app, lane, move).exit_ft

    def clearance_target_ft(self, app: int, lane: int, move: int) -> float:
        """Distance at which the vehicle rear has cleared the downstream span."""
        return (self.exit_ft(app, lane, move) + self.vehicle_length_ft
                + self.after_length_ft)

    def shared_cells(self, first: tuple, second: tuple) -> tuple:
        """Cells both movements occupy, as (app, lane, move) triples."""
        a = set(self.route(*first).cell_ids)
        b = set(self.route(*second).cell_ids)
        return tuple(sorted(a & b))

    @property
    def geometry_hash(self) -> str:
        payload = dict(
            version=self.version, n_cells=self.n_cells,
            vehicle_length_ft=self.vehicle_length_ft,
            border_ft=self.border_ft, after_length_ft=self.after_length_ft,
            conditions=[[c.app, c.lane, c.cell, c.move, c.lb_ft, c.ub_ft]
                        for c in self.conditions],
        )
        return fingerprint(payload)


def resolve_geometry_dir(data_dir) -> Path:
    """Resolve a relative geometry folder against the installed package."""
    path = Path(data_dir)
    return path if path.is_absolute() else PACKAGE_ROOT / path


def load_geometry(physics, data_dir="data/geometry") -> Geometry:
    """Build the shared geometry from the versioned cell table and physics."""
    conditions = read_cell_conditions(physics.n_cells, resolve_geometry_dir(data_dir))
    geometry = Geometry(conditions=conditions, n_cells=physics.n_cells,
                        vehicle_length_ft=physics.vehicle_length_ft,
                        border_ft=physics.border_ft,
                        after_length_ft=physics.after_length_ft,
                        version=physics.geometry_version)
    for app in APPROACHES:
        for lane in LANES:
            for move in legal_moves(lane):
                geometry.route(app, lane, move)
    return geometry
