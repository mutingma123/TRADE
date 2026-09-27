"""RouteCells: the ordered cells one movement occupies, with its route end.

Cells are ordered by the distance at which the front of the vehicle enters
them, which is the order every downstream evaluator assumes when it reads
entry times.
"""

from __future__ import annotations

from dataclasses import dataclass

from .cells import CellCondition, legal_moves


@dataclass(frozen=True)
class RouteCells:
    app: int
    lane: int
    move: int
    cells: tuple

    def __post_init__(self) -> None:
        if not self.cells:
            raise ValueError(f"No geometry for movement {self.app}/{self.lane}/{self.move}")
        if self.move not in legal_moves(self.lane):
            raise ValueError(f"Movement {self.move} is illegal from lane {self.lane}")
        seen = [c.cell for c in self.cells]
        if len(set(seen)) != len(seen):
            raise ValueError("A route visits one cell more than once; intervals would not be connected")

    @property
    def cell_ids(self) -> tuple:
        return tuple(c.cell for c in self.cells)

    @property
    def exit_ft(self) -> float:
        """Travel distance at which the vehicle front leaves its last cell."""
        return max(c.ub_ft for c in self.cells)


def build_route(conditions, app: int, lane: int, move: int) -> RouteCells:
    matched = tuple(sorted((c for c in conditions
                            if c.app == app and c.lane == lane and c.move == move),
                           key=lambda c: (c.lb_ft, c.cell)))
    return RouteCells(app=app, lane=lane, move=move, cells=matched)
