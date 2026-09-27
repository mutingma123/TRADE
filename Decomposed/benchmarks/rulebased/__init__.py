"""Rule-based baselines: all-way stop, first come first served, and actuated.

All three build trajectories on the same kinematics, respect releases, turning
caps, and lane queues, and report completion against the declared common
horizon. All-way stop and first come first served remain distinct control
rules even on instances where they happen to return the same schedule.
"""

from .actuated import actuated
from .allwaystop import allwaystop
from .fcfs import fcfs

RULES = {"allwaystop": allwaystop, "fcfs": fcfs, "actuated": actuated}


def run_rule_controller(name: str, scenario, geometry, physics, benchmark):
    """Dispatch one practical rule on the shared physical instance."""
    if name not in RULES:
        raise ValueError(f"Unknown rule-based controller {name!r}")
    return RULES[name](scenario, geometry, physics, benchmark)

