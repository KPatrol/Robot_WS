"""Unit tests for the detection→navigation safety bridge.

Run:  python -m pytest robots/pi-controller/navigation/test_nav_controller.py

These tests focus on NavController.on_alert() — the seam between the
inline AnomalyDetector and the FSM. We do not exercise the script
executor or line follower here; just the alert routing, cooldowns,
soft-pause window, and EMERGENCY hard-stop.
"""

from __future__ import annotations

import tempfile
import time
import unittest
from unittest import mock

from .nav_controller import Mode, NavController


class _Clock:
    def __init__(self, t: float = 1000.0):
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


def _make_nav() -> NavController:
    return NavController(script_dir=tempfile.mkdtemp(prefix="kpatrol_nav_"))


class FireAlert(unittest.TestCase):
    def test_fire_triggers_emergency_stop(self):
        nav = _make_nav()
        result = nav.on_alert("fire", confidence=0.92)
        self.assertTrue(result["ok"])
        self.assertEqual(result["action"], "emergency_stop")
        self.assertEqual(result["mode"], Mode.EMERGENCY.value)
        self.assertEqual(nav.get_mode(), Mode.EMERGENCY.value)

    def test_fire_emergency_persists_until_clear(self):
        nav = _make_nav()
        nav.on_alert("fire", confidence=0.95)
        # Person alert in EMERGENCY must be ignored, not override.
        result = nav.on_alert("person", confidence=0.80)
        self.assertEqual(result["action"], "ignored_in_emergency")
        self.assertEqual(nav.get_mode(), Mode.EMERGENCY.value)
        # Operator clears.
        nav.clear_emergency()
        self.assertEqual(nav.get_mode(), Mode.MANUAL.value)

    def test_fire_cooldown_blocks_repeats(self):
        # Two fire alerts within the 5s cooldown — second is dropped.
        clock = _Clock(1000.0)
        from . import nav_controller as nc_mod
        with mock.patch.object(nc_mod.time, "monotonic", clock):
            nav = _make_nav()
            r1 = nav.on_alert("fire", confidence=0.9)
            self.assertEqual(r1["action"], "emergency_stop")
            clock.advance(2.0)
            r2 = nav.on_alert("fire", confidence=0.9)
            self.assertEqual(r2["action"], "cooldown")
            self.assertGreater(r2["remaining_s"], 0.0)


class PersonAlert(unittest.TestCase):
    def test_person_pause_in_free_coverage(self):
        clock = _Clock(2000.0)
        from . import nav_controller as nc_mod
        with mock.patch.object(nc_mod.time, "monotonic", clock):
            nav = _make_nav()
            self.assertTrue(nav.set_mode("FREE_COVERAGE"))
            r = nav.on_alert("person", confidence=0.7)
            self.assertEqual(r["action"], "pause")
            self.assertEqual(r["duration_s"], 5.0)
            # Pause window armed but mode preserved.
            self.assertEqual(nav.get_mode(), Mode.FREE_COVERAGE.value)

    def test_person_cooldown_blocks_repeats(self):
        clock = _Clock(3000.0)
        from . import nav_controller as nc_mod
        with mock.patch.object(nc_mod.time, "monotonic", clock):
            nav = _make_nav()
            nav.set_mode("FREE_COVERAGE")
            r1 = nav.on_alert("person", confidence=0.7)
            self.assertEqual(r1["action"], "pause")
            clock.advance(3.0)
            r2 = nav.on_alert("person", confidence=0.8)
            self.assertEqual(r2["action"], "cooldown")
            # 8s person cooldown; 3s elapsed → ~5s remaining.
            self.assertAlmostEqual(r2["remaining_s"], 5.0, delta=0.1)

    def test_unknown_alert_kind_is_noop(self):
        nav = _make_nav()
        r = nav.on_alert("ufo", confidence=0.5)
        self.assertEqual(r["action"], "noop")


class LastAlertSnapshot(unittest.TestCase):
    def test_last_alert_records_kind_and_confidence(self):
        nav = _make_nav()
        nav.on_alert("person", confidence=0.66, bbox=(10, 20, 30, 40))
        snap = nav.get_last_alert()
        self.assertIsNotNone(snap)
        self.assertEqual(snap["kind"], "person")
        self.assertAlmostEqual(snap["confidence"], 0.66, places=4)
        self.assertEqual(snap["bbox"], (10, 20, 30, 40))


if __name__ == "__main__":
    unittest.main()
