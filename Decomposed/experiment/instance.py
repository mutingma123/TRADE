"""Prepare one ladder instance: its packaged geometry and seeded vehicle batch."""

from ..geometry.intersection import load_geometry
from ..scenario.generator import build_scenario


def prepare_instance(config, case, seed):
    geometry = load_geometry(config.physics, config.geometry_data_dir)
    scenario = build_scenario(case, seed, config.physics, geometry)
    return scenario, geometry
