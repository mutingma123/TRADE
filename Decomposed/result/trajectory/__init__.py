"""Trajectory: one canonical schema for every controller and every solve.

States keep per-vehicle release times and omit anything before release.
Acceleration and deceleration are one signed field. Occupancy intervals come
from interpolating between bracketing states at the geometry thresholds, and a
boundary that no pair of states brackets is reported missing rather than
replaced by an endpoint.
"""
