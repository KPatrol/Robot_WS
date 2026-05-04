"""Shared math/numeric helpers for the navigation modes.

Each autonomous mode (free coverage, line follow, GPS waypoint) was carrying
its own copy of these helpers under slightly different names. Centralising them
keeps clamp/wrap/round semantics identical across modes — important when
controller gains and saturation limits are tuned together against the same
hardware envelope.
"""

from __future__ import annotations

import math
from typing import Optional


def clamp(value: float, lo: float, hi: float) -> float:
    """Saturate ``value`` into the inclusive range ``[lo, hi]``.

    Used everywhere we feed motor commands or PD outputs through a hardware
    limit (e.g. ±127 for the firmware's signed-byte velocity protocol).
    """
    return max(lo, min(hi, value))


def wrap_pi(angle_rad: float) -> float:
    """Wrap an angle in radians into ``(-π, π]``.

    Heading errors are computed as differences of two angles and can land
    outside the principal range; without wrapping, a +179° → -179° transition
    would look like a 358° error and spin the robot the wrong way.
    """
    while angle_rad > math.pi:
        angle_rad -= 2 * math.pi
    while angle_rad < -math.pi:
        angle_rad += 2 * math.pi
    return angle_rad


def safe_round(value: float, ndigits: int) -> Optional[float]:
    """Round to ``ndigits`` places, but return ``None`` for NaN/Inf.

    Diagnostic dictionaries are JSON-serialised and shipped over MQTT;
    ``float('nan')`` would raise on the consumer side, so we surface "no
    data" explicitly instead.
    """
    if not math.isfinite(value):
        return None
    return round(value, ndigits)


__all__ = ["clamp", "wrap_pi", "safe_round"]
