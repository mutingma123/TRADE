"""One vehicle's states on the shared discrete kinematics, advanced step by step."""

from __future__ import annotations

from ..result.trajectory.state import State


class Path:
    """Record the current state with its acceleration, then advance one step."""

    def __init__(self, vehicle, physics):
        self.vehicle, self.physics, self.states = vehicle, physics, []
        self.clock = dict(time_s=vehicle.release_s, position_ft=vehicle.ini_dis_ft,
                          speed_fps=vehicle.ini_speed_fps)

    def step(self, acc: float) -> None:
        clock, dt = self.clock, self.physics.dt_s
        self.states.append(State(vid=self.vehicle.vid, time_s=round(clock["time_s"], 8),
                                 position_ft=clock["position_ft"], speed_fps=clock["speed_fps"],
                                 acc_fps2=acc,
                                 before_border=int(clock["position_ft"] < self.physics.border_ft)))
        clock["position_ft"] += clock["speed_fps"] * dt + 0.5 * acc * dt * dt
        clock["speed_fps"] = max(0.0, clock["speed_fps"] + acc * dt)
        clock["time_s"] += dt

    def leave(self, cap: float, geometry) -> list:
        """Hold the cap until the rear clears the downstream span, then one more step."""
        vehicle = self.vehicle
        target = geometry.clearance_target_ft(vehicle.app, vehicle.lane, vehicle.move)
        while self.clock["position_ft"] < target + cap * self.physics.dt_s:
            self.step(0.0)
        self.step(0.0)
        return self.states
