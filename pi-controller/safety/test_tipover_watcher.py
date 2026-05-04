"""Unit tests for TipOverWatcher.

Run:  python -m pytest robots/pi-controller/safety/test_tipover_watcher.py
"""

from __future__ import annotations

import unittest

from .tipover_watcher import TipOverWatcher, TipOverConfig


class _Clock:
    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


class TipOverFiring(unittest.TestCase):
    def test_below_threshold_never_fires(self):
        clock = _Clock()
        fired: list[tuple[str, float]] = []
        w = TipOverWatcher(
            on_tipover=lambda axis, ang: fired.append((axis, ang)),
            config=TipOverConfig(angle_deg=30.0, min_dwell_sec=0.4),
            clock=clock,
        )
        for _ in range(10):
            w.tick(roll_deg=10.0, pitch_deg=5.0)
            clock.advance(0.1)
        self.assertEqual(fired, [])

    def test_dwell_required_before_firing(self):
        clock = _Clock()
        fired: list[tuple[str, float]] = []
        w = TipOverWatcher(
            on_tipover=lambda axis, ang: fired.append((axis, ang)),
            config=TipOverConfig(angle_deg=30.0, min_dwell_sec=0.4),
            clock=clock,
        )
        # First tick crosses the threshold but dwell is 0 — no fire.
        w.tick(45.0, 0.0)
        self.assertEqual(fired, [])
        clock.advance(0.2)
        w.tick(45.0, 0.0)
        self.assertEqual(fired, [])
        clock.advance(0.3)  # total dwell now 0.5s > 0.4s
        w.tick(45.0, 0.0)
        self.assertEqual(len(fired), 1)
        axis, ang = fired[0]
        self.assertEqual(axis, "roll")
        self.assertAlmostEqual(ang, 45.0, places=2)

    def test_pitch_axis_fires_when_larger(self):
        clock = _Clock()
        fired: list[tuple[str, float]] = []
        w = TipOverWatcher(
            on_tipover=lambda axis, ang: fired.append((axis, ang)),
            config=TipOverConfig(angle_deg=30.0, min_dwell_sec=0.0),
            clock=clock,
        )
        # First tick arms dwell; second tick (after clock advance) fires.
        w.tick(31.0, 50.0)
        clock.advance(0.01)
        w.tick(31.0, 50.0)
        self.assertEqual(len(fired), 1)
        self.assertEqual(fired[0][0], "pitch")

    def test_recovery_clears_dwell(self):
        clock = _Clock()
        fired: list[tuple[str, float]] = []
        w = TipOverWatcher(
            on_tipover=lambda axis, ang: fired.append((axis, ang)),
            config=TipOverConfig(angle_deg=30.0, min_dwell_sec=0.4),
            clock=clock,
        )
        w.tick(40.0, 0.0)  # arm
        clock.advance(0.2)
        w.tick(5.0, 0.0)   # recover before dwell met
        clock.advance(0.5)
        w.tick(40.0, 0.0)  # re-arm dwell
        clock.advance(0.2)
        w.tick(40.0, 0.0)
        self.assertEqual(fired, [])  # dwell still not met

    def test_cooldown_prevents_double_fire(self):
        clock = _Clock()
        fired: list[tuple[str, float]] = []
        w = TipOverWatcher(
            on_tipover=lambda axis, ang: fired.append((axis, ang)),
            config=TipOverConfig(angle_deg=30.0, min_dwell_sec=0.0,
                                 fire_cooldown_sec=5.0),
            clock=clock,
        )
        w.tick(40.0, 0.0)
        clock.advance(0.01)
        w.tick(40.0, 0.0)  # fires
        self.assertEqual(len(fired), 1)
        clock.advance(1.0)
        w.tick(40.0, 0.0)  # still tipped, but inside cooldown — no second fire
        self.assertEqual(len(fired), 1)
        clock.advance(5.0)
        # Fully past cooldown — but firing requires fresh dwell after recovery.
        w.tick(5.0, 0.0)
        clock.advance(0.1)
        w.tick(40.0, 0.0)
        clock.advance(0.1)
        w.tick(40.0, 0.0)
        self.assertEqual(len(fired), 2)


class RecoveryCallback(unittest.TestCase):
    def test_on_recover_called_after_fire(self):
        clock = _Clock()
        fired_axes: list[str] = []
        recovers: list[int] = []
        w = TipOverWatcher(
            on_tipover=lambda axis, ang: fired_axes.append(axis),
            on_recover=lambda: recovers.append(1),
            config=TipOverConfig(angle_deg=30.0, min_dwell_sec=0.0),
            clock=clock,
        )
        w.tick(45.0, 0.0)
        clock.advance(0.01)
        w.tick(45.0, 0.0)
        self.assertEqual(len(fired_axes), 1)
        # Robot lifted upright again.
        clock.advance(0.5)
        w.tick(0.0, 0.0)
        self.assertEqual(len(recovers), 1)


if __name__ == "__main__":
    unittest.main()
