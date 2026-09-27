"""Extract physical schedules and exact criteria."""

from ..motion.mode_schedule import materialize

def extract_stage1(stage, scenario):
    schedules = {}
    for v in scenario.vehicles:
        index = max(range(len(stage.zin[v.vid])), key=lambda k: stage.zin[v.vid][k].X)
        schedules[v.vid] = materialize(stage.modes[v.vid][index], stage.entry[v.vid].X)
    proxy = sum(min(stage.caps[v.vid], stage.distances[v.vid] / (schedules[v.vid]['entry_s'] - v.release_s))
                for v in scenario.vehicles) / scenario.n_vehicles
    hinge = sum(max(0.0, stage.crossing_window_s - abs(stage.tin[a, cell].X - stage.tin[b, cell].X))
                for a, b, cell in stage.penalties)
    return schedules, dict(exact_speed_proxy_fps=proxy, event_hinge_s=hinge,
                           reward_lower_bound_fps=stage.model.ObjVal,
                           reward_error_bound_fps=stage.reward_error_bound_fps)
