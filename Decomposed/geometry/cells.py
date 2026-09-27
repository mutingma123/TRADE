"""The published cell-occupancy table, read in its own header convention.

Each row says that a vehicle on approach `app`, lane `lane`, performing
movement `move`, occupies intersection cell `cell` while its one dimensional
travel distance lies in [X_L, X_U] feet. Movement labels follow the input
geometry: move 2 is the wide left sweep through the four central cells, move 1
the tight right turn touching one or two corner cells.
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path

MOVEMENTS = {0: "through", 1: "right", 2: "left"}
LANE_MOVES = {1: (0, 1), 2: (0, 2)}


@dataclass(frozen=True)
class CellCondition:
    app: int
    lane: int
    cell: int
    move: int
    lb_ft: float
    ub_ft: float

    def __post_init__(self) -> None:
        if self.ub_ft <= self.lb_ft:
            raise ValueError(f"Cell {self.cell} has an empty distance interval")
        if self.move not in MOVEMENTS:
            raise ValueError(f"Unknown movement label {self.move}")


def legal_moves(lane: int) -> tuple:
    """Movements the input geometry permits from one lane."""
    if lane not in LANE_MOVES:
        raise ValueError(f"Lane {lane} is outside the published two-lane approach")
    return LANE_MOVES[lane]


def read_cell_conditions(n_cells: int, data_dir) -> tuple:
    """Load `{n_cells}cellcondition_all.csv` from the versioned data folder."""
    path = Path(data_dir) / f"{n_cells}cellcondition_all.csv"
    with path.open(newline="") as handle:
        rows = tuple(
            CellCondition(app=int(r["app"]), lane=int(r["lane"]), cell=int(r["cell"]),
                          move=int(r["move"]), lb_ft=float(r["X_L"]), ub_ft=float(r["X_U"]))
            for r in csv.DictReader(handle)
        )
    if not rows:
        raise ValueError(f"No cell conditions in {path}")
    if max(c.cell for c in rows) != n_cells:
        raise ValueError(f"{path} does not cover exactly {n_cells} cells")
    return rows
