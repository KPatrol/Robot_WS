"""Unit tests for BatteryWatcher.

Run:  python -m pytest robots/pi-controller/safety/test_battery_watcher.py
"""

from __future__ import annotations

import unittest

from .battery_watcher import BatteryWatcher, BatteryConfig


class _Clock:
    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


class Thresholds(unittest.TestCase):
    def test_normal_does_not_fire(self):
        clock = _Clock()
        events: list = []
        w = BatteryWatcher(
            on_event=lambda lvl, pct: events.append((lvl, pct)),
            config=BatteryConfig(),
            clock=clock,
        )
        self.assertIsNone(w.tick(80.0))
        self.assertEqual(events, [])

    def test_low_threshold_fires_once(self):
        clock = _Clock()
        events: list = []
        w = BatteryWatcher(
            on_event=lambda lvl, pct: events.append((lvl, pct)),
            config=BatteryConfig(low_pct=25.0, critical_pct=10.0,
                                 hysteresis_pct=3.0, refire_interval_sec=60.0),
            clock=clock,
        )
        # Cross into LOW.
        w.tick(20.0)
        # Stay LOW — must not re-fire within refire interval.
        clock.advance(5.0)
        w.tick(18.0)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0][0], "low")

    def test_critical_promotes_immediately(self):
        clock = _Clock()
        events: list = []
        w = BatteryWatcher(
            on_event=lambda lvl, pct: events.append((lvl, pct)),
            clock=clock,
        )
        w.tick(20.0)        # LOW
        w.tick(8.0)         # CRITICAL — promotes regardless of refire interval
        self.assertEqual([e[0] for e in events], ["low", "critical"])

    def test_recovery_uses_hysteresis(self):
        clock = _Clock()
        events: list = []
        w = BatteryWatcher(
            on_event=lambda lvl, pct: events.append((lvl, pct)),
            config=BatteryConfig(low_pct=25.0, critical_pct=10.0,
                                 hysteresis_pct=3.0),
            clock=clock,
        )
        w.tick(20.0)  # LOW
        # Climb back to 26% — within hysteresis band, not yet recovered.
        w.tick(26.0)
        self.assertEqual(len(events), 1)
        # 28%+ clears LOW back to NORMAL.
        w.tick(28.5)
        self.assertEqual([e[0] for e in events], ["low", "normal"])

    def test_refire_after_interval(self):
        clock = _Clock()
        events: list = []
        w = BatteryWatcher(
            on_event=lambda lvl, pct: events.append((lvl, pct)),
            config=BatteryConfig(low_pct=25.0, critical_pct=10.0,
                                 refire_interval_sec=30.0),
            clock=clock,
        )
        w.tick(20.0)
        clock.advance(31.0)
        w.tick(19.0)
        self.assertEqual(len(events), 2)
        self.assertEqual([e[0] for e in events], ["low", "low"])

    def test_invalid_pct_is_dropped(self):
        clock = _Clock()
        events: list = []
        w = BatteryWatcher(
            on_event=lambda lvl, pct: events.append((lvl, pct)),
            clock=clock,
        )
        w.tick(-5)
        w.tick(150)
        w.tick("nan")  # type: ignore[arg-type]
        self.assertEqual(events, [])


if __name__ == "__main__":
    unittest.main()
