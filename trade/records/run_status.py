"""The status.json heartbeat of one instance attempt, and its UTC timestamps."""

import os
import socket
from datetime import datetime, timezone
from pathlib import Path

from Decomposed.result.artifacts import write_json


def utc_now():
    return datetime.now(timezone.utc).isoformat(timespec="seconds")


class Status:
    """status.json: state, phase, and heartbeat, read when resuming and by trade check."""

    def __init__(self, root, **fields):
        self.path = Path(root) / "status.json"
        self.record = dict(fields, state="RUNNING", phase="start", pid=os.getpid(),
                           host=socket.gethostname(), started_at=utc_now(),
                           updated_at=utc_now(), caps_done=[])
        write_json(self.path, self.record)

    def update(self, **fields):
        self.record.update(fields, updated_at=utc_now())
        write_json(self.path, self.record)
