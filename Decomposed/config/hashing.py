"""Stable identities for geometry, scenarios, and random streams.

A fingerprint is a SHA-256 digest of canonical JSON, so it is stable across
processes and machines. Random streams are derived from the scenario seed and
a named purpose, which keeps the initial state, arrival, and movement draws
independent of each other.
"""

from __future__ import annotations

import hashlib
import json
from typing import Any


def fingerprint(value: Any) -> str:
    """SHA-256 of canonical JSON; rejects NaN so identities stay comparable."""
    payload = json.dumps(value, sort_keys=True, allow_nan=False, separators=(",", ":"))
    return hashlib.sha256(payload.encode()).hexdigest()


def stream_seed(seed: int, stream: str, *keys: Any) -> int:
    """Derive an independent 64-bit stream seed from a scenario seed.

    `stream` names the purpose: "state", "arrival", or "movement". Keys
    separate lanes, queue positions, and any other index inside one stream.
    """
    payload = json.dumps([seed, stream, *keys], separators=(",", ":"))
    return int.from_bytes(hashlib.sha256(payload.encode()).digest()[:8], "big")
