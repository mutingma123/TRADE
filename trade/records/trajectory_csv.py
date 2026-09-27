"""Gzip CSV of a controller's sampled trajectory."""

import csv
import gzip
import os
from pathlib import Path

from trade.configuration import TRAJECTORY_FIELDS


def write_trajectory_gz(path, trajectory):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    with gzip.open(temporary, "wt", newline="") as handle:
        writer = csv.writer(handle, lineterminator="\n")
        writer.writerow(TRAJECTORY_FIELDS)
        for vid in trajectory.vehicle_ids():
            for state in trajectory.points(vid):
                writer.writerow((vid, f"{state.time_s:.6f}", f"{state.position_ft:.6f}",
                                 f"{state.speed_fps:.6f}", f"{state.acc_fps2:.6f}"))
    os.replace(temporary, path)
