"""Jerk-limited physical motion construction."""

import math

class Motion:
    def __init__(self, vehicle):
        self.t, self.x, self.v, self.a = vehicle.release_s, vehicle.ini_dis_ft, vehicle.ini_speed_fps, 0.0
        self.pieces = []

    def add(self, duration, jerk=0.0):
        if duration < 1e-10:
            return
        power = [self.x, self.v * duration, self.a * duration ** 2 / 2, jerk * duration ** 3 / 6]
        coefficients = [sum(power[j] * math.comb(k, j) / math.comb(5, j)
                            for j in range(min(k, 3) + 1)) for k in range(6)]
        self.pieces.append(dict(start_s=self.t, end_s=self.t + duration, coefficients_ft=coefficients))
        self.x += self.v * duration + self.a * duration ** 2 / 2 + jerk * duration ** 3 / 6
        self.v += self.a * duration + jerk * duration ** 2 / 2
        self.a += jerk * duration
        self.t += duration

    def transition(self, speed, physics, *, free_initial=False):
        delta = speed - self.v
        if abs(delta) < 1e-8:
            return
        sign = 1 if delta > 0 else -1
        limit = physics.max_acc_fps2 if delta > 0 else -physics.max_dec_fps2
        jerk = physics.max_acc_change_fps2 / physics.dt_s
        if free_initial:
            peak = min(limit, math.sqrt(2 * jerk * abs(delta)))
            self.a = sign * peak
            self.add(max(0.0, abs(delta) / peak - peak / (2 * jerk)))
            self.add(peak / jerk, -sign * jerk)
        else:
            peak = min(limit, math.sqrt(jerk * abs(delta)))
            self.add(peak / jerk, sign * jerk)
            self.add(max(0.0, abs(delta) / peak - peak / jerk))
            self.add(peak / jerk, -sign * jerk)
        self.v, self.a = speed, 0.0
