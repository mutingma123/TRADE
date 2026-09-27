"""Rate-limited progress snapshots for unattended monitoring."""

import time
from gurobipy import GRB

from ..result.artifacts import write_json
from .quality import finite


class Progress:
    def __init__(self, path):
        self.path, self.updated, self.error = path, 0.0, None

    def __call__(self, model, where):
        if where != GRB.Callback.MIP or time.monotonic() - self.updated < 5:
            return
        self.updated = time.monotonic()
        try:
            write_json(self.path, dict(
                elapsed_s=model.cbGet(GRB.Callback.RUNTIME),
                incumbent_objective=finite(model.cbGet(GRB.Callback.MIP_OBJBST)),
                objective_bound=finite(model.cbGet(GRB.Callback.MIP_OBJBND)),
                nodes=model.cbGet(GRB.Callback.MIP_NODCNT)))
        except Exception as error:
            self.error = error
            model.terminate()
