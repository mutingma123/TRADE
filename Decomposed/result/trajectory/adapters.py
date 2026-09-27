"""Convert a simulated controller schedule into the canonical trajectory schema.

The adapter reads the controller's own rows and emits States on the global
clock, starting at each vehicle's release. Availability comes from the
clearance records, so a controller that leaves a vehicle short is labeled
partial rather than reported as a complete schedule.
"""

from __future__ import annotations

from .clearance import availability_of, clearance_of
from .state import State, Trajectory


def from_controller_result(result, scenario, geometry, physics, label: str = None) -> Trajectory:
    """Adapt one simulated controller schedule into the shared schema."""
    trajectory = Trajectory(label=label or result.controller, scenario_hash=scenario.scenario_hash,
                            geometry_hash=geometry.geometry_hash, source="benchmark",
                            dt_s=physics.dt_s)
    for row in result.states:
        vid = row["vid"]
        trajectory.states.setdefault(vid, []).append(State(
            vid=vid, time_s=row["time_s"], position_ft=row["position_ft"],
            speed_fps=row["speed_fps"], acc_fps2=row["acc_fps2"],
            before_border=int(row["position_ft"] < physics.border_ft)))
    for rows in trajectory.states.values():
        rows.sort(key=lambda state: state.time_s)
    trajectory.availability = availability_of(clearance_of(trajectory, scenario, geometry),
                                              len(scenario.vehicles))
    return trajectory
