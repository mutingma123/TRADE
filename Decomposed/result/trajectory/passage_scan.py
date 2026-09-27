"""Find all increasing segment crossings without rescanning each trajectory."""

from bisect import bisect_left, bisect_right


def nonmonotone_passage_times(points, targets, tolerance):
    """Preserve first and last crossings for ascending targets and noisy positions."""
    targets = list(targets)
    first, last = [None] * len(targets), [None] * len(targets)
    for before, after in zip(points, points[1:]):
        if after.position_ft <= before.position_ft:
            continue
        lower = bisect_left(targets, before.position_ft)
        upper = bisect_right(targets, after.position_ft)
        for index in range(lower, upper):
            fraction = ((targets[index] - before.position_ft)
                        / (after.position_ft - before.position_ft))
            value = before.time_s + fraction * (after.time_s - before.time_s)
            if first[index] is None:
                first[index] = value
            last[index] = value
    for index, target in enumerate(targets):
        at_start = abs(points[0].position_ft - target) <= tolerance
        if not at_start and (points[0].position_ft > target
                             or points[-1].position_ft < target - tolerance):
            first[index] = last[index] = None
        elif at_start:
            first[index] = points[0].time_s
            if last[index] is None:
                last[index] = first[index]
    return list(zip(first, last))
