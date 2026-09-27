"""Shared physical-cost components after producer-specific validation.

Every record ends at each vehicle's clearance, so a longer record is never
charged for downstream rear exposure that a shorter record does not cover.
"""

from dataclasses import replace

from Decomposed.cost.delay.record import delay_cost
from Decomposed.cost.pricing.prices import PricingConfig
from Decomposed.cost.rearend.collision import rear_cost
from Decomposed.cost.rearend.headway import following_margins
from Decomposed.result.trajectory.clearance import clearance_of
from Decomposed.result.trajectory.intervals import cell_intervals
from Decomposed.result.trajectory.state import Trajectory
from Decomposed.stochastic_timing_error.expectation import crossing_cost

from .polynomial.bernstein import derivative_coefficients
from .polynomial.extrema import power_polynomial


def trim_at_clearance(trajectory, scenario, geometry):
    states = {}
    for record in clearance_of(trajectory, scenario, geometry):
        if record.clearance_s is None:
            raise ValueError('Cannot compare an incomplete trajectory')
        points = trajectory.points(record.vid)
        selected = [row for row in points if row.time_s <= record.clearance_s + 1e-9]
        if selected[-1].time_s < record.clearance_s - 1e-9:
            before = selected[-1]
            after = next(row for row in points if row.time_s > record.clearance_s)
            fraction = (record.clearance_s - before.time_s) / (after.time_s - before.time_s)
            selected.append(replace(before, time_s=record.clearance_s, position_ft=record.target_ft,
                                    speed_fps=before.speed_fps + fraction * (after.speed_fps - before.speed_fps),
                                    acc_fps2=before.acc_fps2, before_border=0))
        else:
            selected[-1] = replace(selected[-1], position_ft=record.target_ft)
        states[record.vid] = selected
    return Trajectory(label=trajectory.label, scenario_hash=trajectory.scenario_hash,
                      geometry_hash=trajectory.geometry_hash, states=states,
                      availability='complete', source=trajectory.source, dt_s=trajectory.dt_s)


def comparable_cost(trajectory, scenario, geometry, physics, *, validated, sigma_s=1.0):
    if not validated:
        raise ValueError('The producing formulation must validate the full trajectory first')
    trimmed = trim_at_clearance(trajectory, scenario, geometry)
    pricing = PricingConfig(name='public')
    clearances = clearance_of(trimmed, scenario, geometry)
    delay = delay_cost(scenario, clearances, geometry, physics, pricing)
    crossing = crossing_cost(cell_intervals(trimmed, scenario, geometry), scenario, sigma_s, pricing.crossing_per_pair)
    rear = rear_cost(following_margins(trimmed, scenario, physics), sigma_s, pricing.rear_per_pair)
    return dict(mean_delay_s=delay.mean_delay_s, total_delay_s=delay.total_delay_s,
                expected_crossing_pairs=crossing.expected_pairs, expected_rear_pairs=rear.expected_pairs,
                delay_cost_usd=delay.delay_cost_usd, crossing_cost_usd=crossing.cost,
                rear_cost_usd=rear.cost, total_cost_usd=delay.delay_cost_usd + crossing.cost + rear.cost,
                sigma_s=sigma_s, lifetime='release through individual clearance')


def continuous_acceleration_variation(polynomials):
    total = 0.0
    for record in polynomials['vehicles'].values():
        for piece in record['segments']:
            duration = piece['end_s'] - piece['start_s']
            jerk = power_polynomial(derivative_coefficients(piece['coefficients_ft'], duration, 3))
            roots = [float(r.real) for r in jerk.roots() if abs(r.imag) < 1e-7 and 0 < r.real < 1]
            knots = sorted([0.0, 1.0] + roots)
            integral = jerk.integ()
            total += duration * sum(abs(integral(b) - integral(a)) for a, b in zip(knots, knots[1:]))
    return float(total)
