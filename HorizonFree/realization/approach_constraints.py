"""Continuous derivative bounds, joins, and endpoint rows for the LP."""

from gurobipy import GRB
from ..polynomial.bernstein import derivative_coefficients

DEGREE = 5

def add_approach_constraints(model, vehicle, physics, knots, cap, cap_time, boundary, speed, entry_acc, segments, objective, jerk_limit):
    vid = vehicle.vid
    previous = None
    for index, (start, finish) in enumerate(zip(knots, knots[1:])):
        duration = finish - start
        if duration < 1e-8:
            continue
        coeff = [model.addVar(lb=-GRB.INFINITY, name=f"b_{vid}_{index}_{k}")
                 for k in range(DEGREE + 1)]
        velocity = derivative_coefficients(coeff, duration)
        acceleration = derivative_coefficients(coeff, duration, 2)
        jerk = derivative_coefficients(coeff, duration, 3)
        upper_speed = (cap if vehicle.is_turning and start >= cap_time - 1e-9
                       else physics.max_speed_through_fps)
        for k, expr in enumerate(velocity):
            model.addConstr(expr >= physics.min_speed_fps, name=f"vmin_{vid}_{index}_{k}")
            model.addConstr(expr <= upper_speed, name=f"vmax_{vid}_{index}_{k}")
        for k, expr in enumerate(acceleration):
            model.addConstr(expr >= physics.max_dec_fps2, name=f"amin_{vid}_{index}_{k}")
            model.addConstr(expr <= physics.max_acc_fps2, name=f"amax_{vid}_{index}_{k}")
        for k, expr in enumerate(jerk):
            model.addConstr(expr >= -jerk_limit, name=f"jmin_{vid}_{index}_{k}")
            model.addConstr(expr <= jerk_limit, name=f"jmax_{vid}_{index}_{k}")
            absolute = model.addVar(lb=0.0, name=f"absj_{vid}_{index}_{k}")
            model.addConstr(absolute >= expr)
            model.addConstr(absolute >= -expr)
            objective.append(duration * absolute / len(jerk))
        if previous is None:
            model.addConstr(coeff[0] == vehicle.ini_dis_ft, name=f"initial_x_{vid}")
            model.addConstr(velocity[0] == vehicle.ini_speed_fps, name=f"initial_v_{vid}")
        else:
            old_coeff, old_duration = previous
            for order in range(3):
                model.addConstr(derivative_coefficients(old_coeff, old_duration, order)[-1]
                                == derivative_coefficients(coeff, duration, order)[0],
                                name=f"join_{vid}_{index}_{order}")
        segments[vid].append(dict(start_s=float(start), end_s=float(finish),
                                  coefficients_ft=coeff))
        previous = coeff, duration
    coeff, duration = previous
    model.addConstr(coeff[-1] == boundary, name=f"entry_x_{vid}")
    model.addConstr(derivative_coefficients(coeff, duration)[-1] == speed,
                    name=f"entry_v_{vid}")
    model.addConstr(derivative_coefficients(coeff, duration, 2)[-1] == entry_acc,
                    name=f"entry_a_{vid}")
    return None
