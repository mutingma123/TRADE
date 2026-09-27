"""Recover occupancy events independently from polynomial roots."""

from ..polynomial.extrema import first_crossing

def check_events(vid, pieces, routes, schedule, fixed_passage, tolerance, failures, intervals):
    for threshold in routes[vid]:
        observed = []
        for kind, distance in (("front entry", threshold.front_entry_ft), ("rear exit", threshold.rear_exit_ft)):
            actual = first_crossing(pieces, distance, tolerance)
            cell_events = schedule.get("cell_events", {})
            cell_event = cell_events.get(threshold.cell, cell_events.get(str(threshold.cell)))
            if cell_event is not None:
                expected = cell_event["entry_s" if kind == "front entry" else "exit_s"]
            elif "cell_entries_s" in schedule and "cell_exits_s" in schedule:
                table = schedule["cell_entries_s" if kind == "front entry" else "cell_exits_s"]
                expected = table.get(threshold.cell, table.get(str(threshold.cell)))
            else:
                expected = first_crossing(fixed_passage, distance, tolerance)
            if actual is None or expected is None or abs(actual - expected) > tolerance:
                failures.append(f"vehicle {vid}, cell {threshold.cell}: missed scheduled {kind}")
            observed.append(actual)
        if None not in observed:
            intervals.append(dict(vid=vid, cell=threshold.cell, entry_s=observed[0], exit_s=observed[1]))
    return None
