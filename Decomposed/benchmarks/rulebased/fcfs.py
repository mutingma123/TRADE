"""First come first served: arrival-ordered reservation without a stop.

Requests are granted in unimpeded boundary arrival order and the intersection
is reserved as a whole, but no vehicle is required to stop. This remains a
different control rule from the all-way stop even on instances where the two
happen to produce the same schedule.
"""

from __future__ import annotations

from ..interface import ControllerProvenance
from .dispatch import serve_in_order

PROVENANCE = ControllerProvenance(
    controller="fcfs",
    algorithm="Arrival-ordered whole-intersection reservation, no mandatory stop",
    source="Standard reservation baseline with declared parameters",
    adaptation="Request order from unimpeded boundary arrival, ties by vehicle identifier",
    parameter_version="benchmark-v1")


def fcfs(scenario, geometry, physics, benchmark):
    """Reserve the intersection in request order, without stopping."""
    result = serve_in_order("fcfs", scenario, geometry, physics, benchmark,
                            whole_box=True)
    result.provenance = PROVENANCE
    return result
