"""Unit tests for AlertActuator.

Run:  python -m pytest robots/pi-controller/safety/test_alert_actuator.py
"""

from __future__ import annotations

import unittest
import time

from .alert_actuator import AlertActuator, ActuatorConfig


class _Recorder:
    def __init__(self):
        self.calls: list[str] = []

    def __call__(self, line: str) -> bool:
        self.calls.append(line)
        return True


class FireAlert(unittest.TestCase):
    def test_fire_emits_strobe_and_alarm(self):
        rec = _Recorder()
        a = AlertActuator(rec)
        a.on_detection("fire", 0.92)
        self.assertIn("LP:WARN_STROBE", rec.calls)
        self.assertIn("BUZZ:ALARM", rec.calls)

    def test_repeat_fire_does_not_resend_same_pattern(self):
        rec = _Recorder()
        a = AlertActuator(rec)
        a.on_detection("fire", 0.92)
        n_first = len(rec.calls)
        a.on_detection("fire", 0.93)
        # Continuous patterns latch — second call must be a no-op.
        self.assertEqual(len(rec.calls), n_first)


class PersonAlert(unittest.TestCase):
    def test_person_emits_blink_and_beep(self):
        rec = _Recorder()
        a = AlertActuator(rec)
        a.on_detection("person", 0.7)
        self.assertIn("LP:WARN_BLINK", rec.calls)
        self.assertIn("BUZZ:BEEP", rec.calls)

    def test_beep_is_redebounced_after_cooldown(self):
        rec = _Recorder()
        cfg = ActuatorConfig(beep_repeat_min_sec=0.0)
        a = AlertActuator(rec, cfg)
        a.on_detection("person", 0.7)
        n1 = len(rec.calls)
        # Force a tiny sleep so the monotonic clock advances past the 0.0
        # threshold; this is a one-shot, so the actuator must re-arm.
        time.sleep(0.01)
        a.on_detection("person", 0.71)
        self.assertGreater(len(rec.calls), n1)


class TipOverAndBattery(unittest.TestCase):
    def test_tipover_emits_sos(self):
        rec = _Recorder()
        a = AlertActuator(rec)
        a.on_tipover("roll", 45.0)
        self.assertIn("LP:SOS", rec.calls)
        self.assertIn("BUZZ:SOS", rec.calls)

    def test_battery_critical_emits_strobe_alarm(self):
        rec = _Recorder()
        a = AlertActuator(rec)
        a.on_battery("critical", 7.0)
        self.assertIn("LP:WARN_STROBE", rec.calls)
        self.assertIn("BUZZ:ALARM", rec.calls)


class FireToOffSequence(unittest.TestCase):
    def test_clear_resets_to_off(self):
        rec = _Recorder()
        a = AlertActuator(rec)
        a.on_detection("fire", 0.95)
        rec.calls.clear()
        a.clear()
        self.assertIn("LP:OFF", rec.calls)
        self.assertIn("BUZZ:OFF", rec.calls)

    def test_clear_is_idempotent(self):
        rec = _Recorder()
        a = AlertActuator(rec)
        a.clear()
        # Initial state was already OFF — must not write anything.
        self.assertEqual(rec.calls, [])


class UnknownKind(unittest.TestCase):
    def test_unknown_kind_is_noop(self):
        rec = _Recorder()
        a = AlertActuator(rec)
        result = a.on_detection("ufo", 0.5)
        self.assertEqual(rec.calls, [])
        self.assertEqual(result, {"light": "OFF", "buzzer": "OFF"})


if __name__ == "__main__":
    unittest.main()
