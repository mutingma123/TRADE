"""Own Gurobi resources through model construction and solving."""

from contextlib import contextmanager
from types import SimpleNamespace

import gurobipy as gp


class Stage(SimpleNamespace):
    def dispose(self):
        try:
            self.model.dispose()
        finally:
            self.env.dispose()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.dispose()


@contextmanager
def constructing_stage(name):
    """Transfer ownership on success; clean up if any construction step fails."""
    env = gp.Env(empty=True)
    model = None
    try:
        env.setParam("OutputFlag", 0)
        env.start()
        model = gp.Model(name, env=env)
        yield Stage(model=model, env=env)
    except BaseException:
        try:
            if model is not None:
                model.dispose()
        finally:
            env.dispose()
        raise
