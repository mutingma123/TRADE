"""Explicit optimality certificates; a feasible incumbent alone is insufficient."""

import math


def finite(value):
    """A real number, or None for NaN, infinity, and Gurobi's 1e100 no-value sentinel."""
    if isinstance(value, (int, float)) and math.isfinite(value) and abs(value) < 1e100:
        return float(value)
    return None


def certificate(result, config):
    incumbent = result.get("incumbent", False)
    absolute = result.get("absolute_gap")
    relative = result.get("mip_gap")
    optimal = incumbent and result["status"] == "OPTIMAL"
    near = incumbent and ((relative is not None and relative <= config.mip_gap + 1e-12)
                          or (absolute is not None and absolute <= config.mip_gap_abs))
    return dict(accepted=bool(optimal or near), label="OPTIMAL" if optimal else
                "NEAR_OPTIMAL" if near else "UNCERTIFIED", relative_target=config.mip_gap,
                absolute_target=config.mip_gap_abs)
