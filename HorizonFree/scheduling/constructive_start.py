"""A constructive zero-exposure incumbent."""

from Decomposed.scenario.leaders import leader_of

def _start(stage, scenario, physics):
    """A single constructive zero-proximity start; no optimization or anchor solve."""
    selected, times, waits, cell_history = {}, {}, {}, {}
    stage.start_complete = False
    for v in sorted(scenario.vehicles, key=lambda v: (v.order, v.release_s, v.vid)):
        vid = v.vid
        leader = leader_of(v, scenario.vehicles)
        rules = None if leader is None else stage.lane_rules[leader.vid, vid]
        candidates = []
        for index, mode in enumerate(stage.modes[vid]):
            lower = mode['entry_s']
            if rules is not None:
                pair = selected[leader.vid], index
                if pair in rules['forbidden']:
                    continue
                if pair in rules['max_lead_wait'] and waits[leader.vid] > rules['max_lead_wait'][pair]:
                    continue
                if pair in rules['min_follow_wait']:
                    lower = max(lower, mode['entry_s'] + rules['min_follow_wait'][pair])
                if pair in rules['offsets']:
                    lower = max(lower, mode['entry_s'] + waits[leader.vid] + rules['offsets'][pair])
            for cell, offset in mode['entries'].items():
                for other, enter, leave in cell_history.get(cell, []):
                    lower = max(lower, leave + stage.gap_s - offset)
                    if other.app != v.app:
                        lower = max(lower, enter + physics.crossing_window_s - offset)
            if leader is not None:
                lower = max(lower, times[leader.vid])
                if leader.move == v.move:
                    lm = stage.modes[leader.vid][selected[leader.vid]]
                    offset = stage.routes[vid][0].front_entry_ft - stage.routes[leader.vid][0].front_entry_ft
                    lower = max(lower, times[leader.vid] + (physics.vehicle_length_ft + physics.safe_buffer_ft
                                                          + offset + lm['speed_deficit_ft']) / lm['cap_fps'])
            if mode['name'] == 'stop' or lower <= mode['entry_s'] + 1e-8:
                actual = max(lower, mode['entry_s'])
                score = min(stage.caps[vid], stage.distances[vid] / (actual - v.release_s))
                candidates.append((-score, actual, index))
        if not candidates:
            return  # Leave a partial start; the solver may still complete it.
        _, entry, index = min(candidates)
        selected[vid], times[vid] = index, entry
        mode = stage.modes[vid][index]
        waits[vid] = max(0.0, entry - mode['entry_s'])
        stage.entry[vid].Start, stage.finish[vid].Start = entry, entry + mode['duration_s']
        stage.waits[vid].Start = waits[vid]
        for k, z in enumerate(stage.zin[vid]):
            z.Start = int(k == index)
        knots, selectors = stage.tangents[vid]
        values = [2 * s - s * s * (entry - v.release_s) / stage.distances[vid] for s in knots]
        best = max(range(len(values)), key=values.__getitem__)
        for k, z in enumerate(selectors):
            z.Start = int(k == best and mode['name'] == 'stop')
        stage.reward[vid].Start = (max(0, min(stage.caps[vid], values[best])) if mode['name'] == 'stop'
                                  else min(stage.caps[vid], stage.distances[vid] / (entry - v.release_s)))
        for cell in mode['entries']:
            enter, leave = entry + mode['entries'][cell], entry + mode['exits'][cell]
            stage.tin[vid, cell].Start, stage.tout[vid, cell].Start = enter, leave
            cell_history.setdefault(cell, []).append((v, enter, leave))
    for (a, b), y in stage.order.items():
        y.Start = int(times[a] < times[b])
    for penalty in stage.penalties.values():
        penalty.Start = 0.0
    for (a, b), y in stage.both_stop.items():
        y.Start = int(stage.modes[a][selected[a]]['name'] == 'stop' and stage.modes[b][selected[b]]['name'] == 'stop')
    stage.start_complete = True
