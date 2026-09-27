"""Where a price came from, in enough detail to audit it.

Every unit cost carries its source, table, publication date, currency year,
severity, and unit. A price without these fields is rejected, because a
number with no provenance cannot be checked and cannot be converted safely
into another currency year.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class PriceProvenance:
    source: str
    table: str
    published: str
    currency_year: int
    severity: str
    unit: str
    note: str = ""

    def __post_init__(self) -> None:
        for field in ("source", "table", "published", "severity", "unit"):
            if not getattr(self, field):
                raise ValueError(f"Price provenance needs a {field}")
        if not 1990 <= self.currency_year <= 2100:
            raise ValueError(f"Implausible currency year {self.currency_year}")
