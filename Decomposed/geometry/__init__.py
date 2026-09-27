"""Geometry: the one owner of lanes, movements, routes, and occupied cells.

Config names the geometry; this component defines it. Shared-cell overlap is
evaluated at the supplied cell resolution and does not assert exact body
contact. Occupancy intervals are assumed connected: a route that would visit a
cell twice is rejected rather than covered by one enclosing interval.
"""
