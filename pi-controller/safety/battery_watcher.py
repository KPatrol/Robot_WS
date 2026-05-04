"""
Battery low / critical watcher.

Consumes a battery percentage stream (from telemetry / firmware ADC) and emits
events when the level crosses two thresholds — `low_pct` and `critical_pct`.
Uses hysteresis so a battery jiggling around the threshold only fires once.

Wiring (kpatrol_mqtt_v5):
    bat = BatteryWatcher(
        on_event=lambda level, pct: (
            actuator.on_battery(level, pct),
            self._publish_battery_alert(level, pct),
        ),
    )
    # in telemetry loop after reading firmware battery_pct:
    bat.tick(battery_pct)
"""

from __future__ import annotations

import logging
import threading
import time
from dataclasses import dataclass
from typing import Callable, Optional

log = logging.getLogger("kpatrol.safety.battery")


@dataclass
class BatteryConfig:
    # Below this percentage we are "low" — slow blink + occasional beep.
    low_pct: float = 25.0
    # Below this percentage we are "critical" — strobe + alarm + must dock.
    critical_pct: float = 10.0
    # Hysteresis: once we've fired LOW we don't clear back to NORMAL until
    # the battery climbs at least this many points above low_pct (e.g. 25 + 3
    # = 28%). Prevents oscillation when the pack rests near the threshold.
    hysteresis_pct: float = 3.0
    # Re-fire the same level no more than this often. Prevents the actuator
    # from being re-armed every telemetry tick.
    refire_interval_sec: float = 60.0


_LEVELS = ("normal", "low", "critical")


class BatteryWatcher:
    def __init__(
        self,
        on_event: Callable[[str, float], None],
        config: Optional[BatteryConfig] = None,
        clock: Callable[[], float] = time.monotonic,
    ):
        self.on_event = on_event
        self.config = config or BatteryConfig()
        self._clock = clock
        self._lock = threading.Lock()
        self._level: str = "normal"
        self._last_fire_ts: float = 0.0

    def tick(self, pct: float) -> Optional[str]:
        """Feed a battery percentage. Returns the level fired this tick or None."""
        try:
            p = float(pct)
        except (TypeError, ValueError):
            return None
        if p < 0 or p > 100:
            return None  # bogus reading from firmware — drop

        cfg = self.config
        now = self._clock()
        new_level = self._classify(p)

        with self._lock:
            cur = self._level
            last = self._last_fire_ts
            # Promote (downgrade battery health) immediately; demote with hysteresis.
            if _rank(new_level) > _rank(cur):
                fire = True
            elif _rank(new_level) < _rank(cur) and self._has_recovered(p, cur):
                fire = True
                self._level = new_level
            elif new_level == cur and new_level != "normal" and now - last >= cfg.refire_interval_sec:
                fire = True
            else:
                fire = False
            if fire:
                self._level = new_level
                self._last_fire_ts = now

        if not fire:
            return None
        log.info("[battery] level=%s pct=%.1f", new_level, p)
        try:
            self.on_event(new_level, p)
        except Exception as exc:
            log.warning("[battery] callback raised: %s", exc)
        return new_level

    def _classify(self, pct: float) -> str:
        if pct < self.config.critical_pct:
            return "critical"
        if pct < self.config.low_pct:
            return "low"
        return "normal"

    def _has_recovered(self, pct: float, cur_level: str) -> bool:
        cfg = self.config
        if cur_level == "critical":
            return pct >= cfg.critical_pct + cfg.hysteresis_pct
        if cur_level == "low":
            return pct >= cfg.low_pct + cfg.hysteresis_pct
        return True

    def level(self) -> str:
        with self._lock:
            return self._level


def _rank(level: str) -> int:
    try:
        return _LEVELS.index(level)
    except ValueError:
        return -1
