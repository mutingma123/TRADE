"""Validate a controller's sampled schedule before reporting comparable costs."""

from .completion import check_completion
from .exclusivity import check_cell_exclusivity, check_interval_shape
from .following import check_following
from .kinematics import check_bounds, check_dynamics, check_turning_caps
from .schema import check_clock, check_finite, check_identities, check_initial_state
from ..result.trajectory.clearance import availability_of, clearance_of
from ..result.trajectory.intervals import cell_intervals


def validate_trajectory(trajectory, scenario, geometry, physics):
    """Return named failures; an incumbent alone is not a feasibility certificate.

    A rule baseline has no planning horizon of its own, so it may clear after
    the declared physics horizon. Every physical check still applies, and the
    overrun is reported as `horizon_extension_s` instead of failing.
    """
    extension_s = 0.0
    checks = {
        "identities": check_identities(trajectory, scenario),
        "finite": check_finite(trajectory),
        "clock": check_clock(trajectory, scenario, physics),
        "initial_state": check_initial_state(trajectory, scenario),
    }
    if trajectory.geometry_hash != geometry.geometry_hash:
        checks["identities"].append("the trajectory uses a different geometry")
    if abs(trajectory.dt_s - physics.dt_s) > 1e-9:
        checks["clock"].append("the trajectory declares a different step length")
    if not any(checks.values()):
        intervals = cell_intervals(trajectory, scenario, geometry)
        clearances = clearance_of(trajectory, scenario, geometry)
        trajectory.availability = availability_of(clearances, scenario.n_vehicles)
        horizon_s = physics.horizon_steps * physics.dt_s
        extension_s = max([0.0] + [r.clearance_s - horizon_s for r in clearances if r.available])
        checks.update(
            completion=check_completion(clearances, scenario),
            occupancy=check_cell_exclusivity(intervals),
            interval_shape=check_interval_shape(intervals),
            following=check_following(trajectory, scenario, physics),
            dynamics=check_dynamics(trajectory, scenario, physics),
            bounds=check_bounds(trajectory, scenario, physics),
            turning=check_turning_caps(trajectory, scenario, physics))
    failures = {name: rows for name, rows in checks.items() if rows}
    return {"valid": not failures, "failures": failures,
            "horizon_extension_s": extension_s if extension_s > physics.dt_s else 0.0}
