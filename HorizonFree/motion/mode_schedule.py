"""Materialize a mode with a continuous waiting duration."""

from .threshold_crossings import portion

def materialize(mode, entry_s, tolerance=1e-5):
    """The schedule and witness of ``mode`` entering at ``entry_s``.

    Solver tolerances leave a Stage 1 entry slightly off its mode's timing.
    An entry within ``tolerance`` is snapped onto the certified family: a
    nonstopping mode enters at its fixed instant, and a shorter wait of a
    stopping mode is zero. Otherwise the passage would not span entry to
    completion, or would start with a sliver piece that float64 cannot
    keep continuous.
    """
    wait = entry_s - mode['entry_s']
    if wait < -tolerance or (mode['wait_start_s'] is None and abs(wait) > tolerance):
        raise ValueError('Mode timing is outside its certified family')
    if mode['wait_start_s'] is None or wait < tolerance:
        wait = 0.0
    entry_s = mode['entry_s'] + wait
    pieces = []
    for original in mode['pieces']:
        shift = wait if mode['wait_start_s'] is not None and original['start_s'] >= mode['wait_start_s'] - 1e-8 else 0.0
        pieces.append(dict(original, start_s=original['start_s'] + shift, end_s=original['end_s'] + shift))
    if wait > 0.0:
        pieces.append(dict(start_s=mode['wait_start_s'], end_s=mode['wait_start_s'] + wait,
                           coefficients_ft=[mode['wait_position_ft']] * 6))
    pieces.sort(key=lambda p: p['start_s'])
    approach_knots = sorted({t for p in pieces for t in (p['start_s'], p['end_s']) if t < entry_s - 1e-8})
    passage = [portion(p, max(p['start_s'], entry_s), p['end_s']) for p in pieces if p['end_s'] > entry_s + 1e-8]
    return dict(entry_s=entry_s, exit_s=entry_s + mode['duration_s'],
                passage_speed_fps=mode['entry_speed_fps'], entry_speed_fps=mode['entry_speed_fps'],
                entry_acc_fps2=mode['entry_acc_fps2'], mode=mode['name'],
                approach_breakpoints_s=approach_knots, passage_segments=passage,
                cell_entries_s={c: entry_s + offset for c, offset in mode['entries'].items()},
                cell_exits_s={c: entry_s + offset for c, offset in mode['exits'].items()},
                witness_segments=pieces)
