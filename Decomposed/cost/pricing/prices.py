"""PricingConfig: versioned delay, crossing, and rear unit costs.

Person-hour time values are converted to vehicle-hours with an explicit
occupancy factor, and every priced term uses one common dollar year. The
crossing and rear-end unit costs stay independently configurable, and the
rear-end cost defaults to the same injury-crash value, because both price an
expected collision under the same arrival error model.
"""

from __future__ import annotations

from dataclasses import dataclass, field

from ...config.validators import positive
from .provenance import PriceProvenance

USDOT_TIME = PriceProvenance(
    source="USDOT Benefit-Cost Analysis Guidance for Discretionary Grant Programs",
    table="Table A-2 value of travel time savings, all purposes; Table A-3 occupancy",
    published="2026 update, December 2025", currency_year=2024,
    severity="not applicable", unit="USD per person-hour, scaled by occupancy",
    note="21.80 USD per person-hour times 1.52 persons per vehicle")

USDOT_CRASH = PriceProvenance(
    source="USDOT Benefit-Cost Analysis Guidance for Discretionary Grant Programs",
    table="Table A-1 monetized value of one crash by severity",
    published="2026 update, December 2025", currency_year=2024,
    severity="injury", unit="USD per crash",
    note="Property damage only 9,700; fatal 15,366,900")


@dataclass(frozen=True)
class PricingConfig:
    name: str = "public"
    currency_year: int = 2024
    delay_per_vehicle_hour: float = 33.14
    crossing_per_pair: float = 342_400.0
    rear_per_pair: float = 342_400.0
    vehicle_occupancy: float = 1.52
    time_provenance: PriceProvenance = field(default_factory=lambda: USDOT_TIME)
    crossing_provenance: PriceProvenance = field(default_factory=lambda: USDOT_CRASH)
    rear_provenance: PriceProvenance = field(default_factory=lambda: USDOT_CRASH)

    def __post_init__(self) -> None:
        positive("delay_per_vehicle_hour", self.delay_per_vehicle_hour)
        positive("vehicle_occupancy", self.vehicle_occupancy)
        for name in ("crossing_per_pair", "rear_per_pair"):
            if getattr(self, name) < 0:
                raise ValueError(f"{name} must be nonnegative")
        years = {self.currency_year, self.time_provenance.currency_year,
                 self.crossing_provenance.currency_year, self.rear_provenance.currency_year}
        if len(years) != 1:
            raise ValueError("Every priced term must use one common dollar year")
