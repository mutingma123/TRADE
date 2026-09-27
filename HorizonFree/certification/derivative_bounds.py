"""Independent extrema checks for speed, acceleration, and jerk."""

import math
from ..polynomial.bernstein import derivative_coefficients, segment_coefficients
from ..polynomial.extrema import polynomial_range, rounding_allowance

def check_derivatives(vehicle, pieces, physics, tolerance, failures):
    vid = vehicle.vid
    jerk_limit = physics.max_acc_change_fps2 / physics.dt_s
    largest_allowance = 0.0
    limits = {1: (physics.min_speed_fps, physics.max_speed_through_fps),
              2: (physics.max_dec_fps2, physics.max_acc_fps2),
              3: (-jerk_limit, jerk_limit)}
    vehicle_extrema = {"speed_min_fps": math.inf, "speed_max_fps": -math.inf,
                       "acc_min_fps2": math.inf, "acc_max_fps2": -math.inf,
                       "jerk_abs_max_fps3": 0.0}
    for index, segment in enumerate(pieces):
        duration = segment["end_s"] - segment["start_s"]
        for order, (lower, upper) in limits.items():
            coefficients = derivative_coefficients(segment["coefficients_ft"], duration, order)
            low, high, _, _ = polynomial_range(coefficients)
            allowance = rounding_allowance(segment["coefficients_ft"], duration, order)
            largest_allowance = max(largest_allowance, allowance)
            if low < lower - tolerance - allowance or high > upper + tolerance + allowance:
                failures.append(f"vehicle {vid}: derivative {order} bounds in segment {index}: [{low}, {high}]")
            if order == 1:
                vehicle_extrema["speed_min_fps"] = min(vehicle_extrema["speed_min_fps"], low)
                vehicle_extrema["speed_max_fps"] = max(vehicle_extrema["speed_max_fps"], high)
            elif order == 2:
                vehicle_extrema["acc_min_fps2"] = min(vehicle_extrema["acc_min_fps2"], low)
                vehicle_extrema["acc_max_fps2"] = max(vehicle_extrema["acc_max_fps2"], high)
            else:
                vehicle_extrema["jerk_abs_max_fps3"] = max(vehicle_extrema["jerk_abs_max_fps3"], abs(low), abs(high))
        cap_start = max(segment["start_s"], vehicle.release_s + 0.5)
        if vehicle.is_turning and cap_start < segment["end_s"] - 1e-10:
            local = segment_coefficients(segment, cap_start, segment["end_s"])
            _, high, _, _ = polynomial_range(derivative_coefficients(local, segment["end_s"] - cap_start))
            if high > physics.max_speed_turn_fps + tolerance:
                failures.append(f"vehicle {vid}: turning speed cap after release + 0.5 s")
    return vehicle_extrema, largest_allowance
