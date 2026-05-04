"""
Tip-over watcher.

Polls the latest IMU sample (BNO08x roll / pitch from motor_controller IMU
status) and triggers EMERGENCY when the magnitude of either axis exceeds a
threshold for `min_dwell_sec` consecutive samples. The dwell guards against
brief jolts (cable bumps, threshold crossings) so we only fire on genuine
falls.

The watcher does not own a thread of its own — it expects the host to call
`tick(roll_deg, pitch_deg)` once per IMU read (10 Hz typical). This keeps
the time source under host control and makes unit tests deterministic.

Wiring (kpatrol_mqtt_v5):
    tip = TipOverWatcher(
        on_tipover=lambda axis, ang: nav.trigger_emergency(f"tipover-{axis}"),
        on_recover=lambda: log.info("[tip] recovered"),
    )
    # in IMU read loop:
    if status: tip.tick(status.roll, status.pitch)
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

log = logging.getLogger("kpatrol.safety.tipover")


@dataclass
class TipOverConfig:
    # Threshold beyond which we consider the robot to be tipping over.
    # 30° is well past "going up a ramp" and well short of "fully fallen".
    angle_deg: float = 30.0
    # Sustained duration above the threshold required to fire — filters out
    # encoder noise and single-frame IMU spikes.
    min_dwell_sec: float = 0.4
    # After firing, do not re-fire for this many seconds even if we still
    # see a tipped angle. The operator must clear EMERGENCY before another
    # event makes sense.
    fire_cooldown_sec: float = 5.0


class TipOverWatcher:
    def __init__(
        self,
        on_tipover: Callable[[str, float], None],
        on_recover: Optional[Callable[[], None]] = None,
        config: Optional[TipOverConfig] = None,
        clock: Callable[[], float] = time.monotonic,
    ):
        self.on_tipover = on_tipover
        self.on_recover = on_recover
        self.config = config or TipOverConfig()
        self._clock = clock
        self._lock = threading.Lock()
        self._above_since: Optional[float] = None
        # `None` means "never fired" — distinct from t=0 so the first
        # tip-over isn't suppressed by the cooldown check.
        self._last_fire_ts: Optional[float] = None
        self._tipped: bool = False

    def tick(self, roll_deg: float, pitch_deg: float) -> Optional[str]:
        """Feed one IMU sample. Returns the fired axis ("roll"/"pitch") or None."""
        try:
            roll = float(roll_deg)
            pitch = float(pitch_deg)
        except (TypeError, ValueError):
            return None
        thr = self.config.angle_deg
        worst_axis: Optional[str] = None
        worst_mag = 0.0
        if abs(roll) >= thr:
            worst_axis = "roll"
            worst_mag = abs(roll)
        if abs(pitch) >= thr and abs(pitch) > worst_mag:
            worst_axis = "pitch"
            worst_mag = abs(pitch)

        now = self._clock()
        with self._lock:
            if worst_axis is None:
                # Recovered — clear dwell tracker so a future tip starts fresh.
                if self._above_since is not None:
                    self._above_since = None
                if self._tipped:
                    self._tipped = False
                    if self.on_recover:
                        try:
                            self.on_recover()
                        except Exception as exc:
                            log.warning("[tipover] on_recover raised: %s", exc)
                return None

            if self._above_since is None:
                self._above_since = now
                return None

            dwell = now - self._above_since
            if dwell < self.config.min_dwell_sec:
                return None
            if (self._last_fire_ts is not None
                    and now - self._last_fire_ts < self.config.fire_cooldown_sec):
                return None
            self._last_fire_ts = now
            self._tipped = True
            axis = worst_axis
            angle = worst_mag

        log.warning("[tipover] firing: axis=%s angle=%.1f° dwell=%.2fs", axis, angle, dwell)
        try:
            self.on_tipover(axis, angle)
        except Exception as exc:
            log.error("[tipover] callback raised: %s", exc)
        return axis

    def is_tipped(self) -> bool:
        with self._lock:
            return self._tipped
