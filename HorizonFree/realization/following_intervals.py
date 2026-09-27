"""Shared polynomial intervals and physical lane gaps."""

from ..polynomial.bernstein import segment_at

def following_intervals(leader, follower, segments, schedules, physics):
    """Common polynomial pieces and required gap, terminating at each vehicle's own completion.

    Diverging movements retain the length gap through follower cell entry,
    a conservative extension of the shared approach by at most the geometry's
    first-cell offset. Different downstream movements are not compared.
    """
    ahead, behind = segments[leader.vid], segments[follower.vid]
    start = max(ahead[0]["start_s"], behind[0]["start_s"])
    end = min(ahead[-1]["end_s"], behind[-1]["end_s"])
    entry = schedules[follower.vid]["entry_s"]
    if leader.move != follower.move:
        end = min(end, entry)
    if end <= start + 1e-10:
        return
    knots = {start, end}
    knots.update(point for pieces in (ahead, behind) for seg in pieces
                 for point in (seg["start_s"], seg["end_s"]) if start < point < end)
    if start < entry < end:
        knots.add(entry)
    knots = sorted(knots)
    for lo, hi in zip(knots, knots[1:]):
        if hi - lo < 1e-9:
            continue
        midpoint = (lo + hi) / 2.0
        gap = physics.vehicle_length_ft
        if leader.move == follower.move and midpoint >= entry:
            gap += physics.safe_buffer_ft
        yield lo, hi, gap, segment_at(ahead, midpoint), segment_at(behind, midpoint)
