"""
Alarm Rule Engine.

Bridges high-level safety events to firmware actuators *with operator-defined
rules*. Where AlertActuator hard-codes "fire → STROBE+ALARM, person → BLINK+BEEP",
this controller lets the operator override per event-type, with optional
time-of-day scheduling and a continuous-duration threshold (the V10 spec calls
for 15s default — a brief detection that immediately disappears should not
wake the buzzer).

Lifecycle
---------
* Rules are pushed from the Web UI → backend → MQTT topic
  `kpatrol/{serial}/alarm/rules` (retained).
* The Pi loads them on boot via the retained payload and updates the in-memory
  store every time a new rules payload arrives.
* On each safety event (`on_event(kind, ...)`) the controller walks active
  rules, matches `event_type`, checks the time window, and starts (or extends)
  a per-rule continuous-fire timer.
* When the timer crosses `continuous_duration_s` the controller dispatches
  the configured light + buzzer pattern via the shared `send_cmd` sink and
  publishes a record onto `kpatrol/{serial}/alarm/triggered` (best effort).

Concurrency
-----------
All mutation runs under a single lock. Action dispatch and the publish hook
fire **outside** the lock so a slow UART or broker cannot block other safety
threads (detector, IMU watcher, battery watcher).

Stale-event self-clear
----------------------
Each rule remembers the wall-clock of its latest matching event. If no new
event arrives within `event_idle_clear_s` (default 5s) the continuous timer
resets — otherwise a person who entered the frame once would keep the rule
"armed" forever and the 15s threshold becomes meaningless.
"""

from __future__ import annotations

import json
import logging
import threading
import time
from dataclasses import dataclass, field
from datetime import datetime
from typing import Any, Callable, Dict, List, Optional, Tuple

log = logging.getLogger("kpatrol.safety.alarm")


# Event types the rule engine understands. Strings (not Enum) for easy
# JSON round-tripping with the backend.
EVENT_PERSON = "person"
EVENT_FIRE = "fire"
EVENT_BATTERY_LOW = "battery_low"
EVENT_BATTERY_CRITICAL = "battery_critical"
EVENT_TIPOVER = "tipover"
EVENT_SCHEDULE = "schedule"   # pure time-of-day trigger (e.g. shift bell)
EVENT_SYSTEM_ERROR = "system_error"
EVENT_ANY_SAFETY = "any_safety"

_VALID_EVENTS = {
    EVENT_PERSON, EVENT_FIRE,
    EVENT_BATTERY_LOW, EVENT_BATTERY_CRITICAL,
    EVENT_TIPOVER, EVENT_SCHEDULE,
    EVENT_SYSTEM_ERROR, EVENT_ANY_SAFETY,
}

# Vocabulary patterns accepted by firmware. We validate at rule-load time so
# a typo in the UI never reaches the UART as garbage.
_LIGHT_PATTERNS = {"OFF", "WARN_BLINK", "WARN_STROBE", "BOTH_BLINK", "SOS"}
_BUZZER_PATTERNS = {"OFF", "ON", "BEEP", "ALARM", "SOS"}


@dataclass
class TimeWindow:
    """One time slot the rule is active in.

    `start_min` / `end_min` are wall-minutes since midnight (0-1440). End may
    wrap past midnight (e.g. 22:00-06:00) by setting end < start.
    `weekdays` is the set of ISO weekdays (1=Mon ... 7=Sun) on which the slot
    applies. Empty set ≡ every day.
    """

    start_min: int = 0
    end_min: int = 1440
    weekdays: frozenset = field(default_factory=frozenset)

    def contains(self, now: datetime) -> bool:
        if self.weekdays and now.isoweekday() not in self.weekdays:
            return False
        cur = now.hour * 60 + now.minute
        s, e = self.start_min, self.end_min
        if s == e:
            return True  # 24/7
        if s < e:
            return s <= cur < e
        # Wraps across midnight: active if cur >= s OR cur < e
        return cur >= s or cur < e

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "TimeWindow":
        if "start" in d and ":" in str(d["start"]):
            s_h, s_m = str(d["start"]).split(":", 1)
            start_min = int(s_h) * 60 + int(s_m)
        else:
            start_min = int(d.get("start_min", 0))
        if "end" in d and ":" in str(d["end"]):
            e_h, e_m = str(d["end"]).split(":", 1)
            end_min = int(e_h) * 60 + int(e_m)
        else:
            end_min = int(d.get("end_min", 1440))
        wd = d.get("weekdays") or []
        return cls(start_min=start_min, end_min=end_min, weekdays=frozenset(int(x) for x in wd))


@dataclass
class AlarmRule:
    """One operator-configured alarm rule.

    `id` is the backend primary key (UUID string). `event_type` is one of
    EVENT_* constants. `windows` is the active time-of-day slots (empty list ≡
    always active). `continuous_duration_s` is the dwell threshold — the
    event must persist for at least this long before the actions fire.

    `actions` are a snapshot of (light_pattern, buzzer_pattern). Either can
    be "NONE" to opt out of that channel.

    `cooldown_s` is the minimum gap between two firings of the same rule —
    once fired we won't re-fire for this many seconds even if the event keeps
    coming in. Prevents the buzzer from re-arming on every detector tick.
    """

    id: str
    name: str
    event_type: str
    enabled: bool = True
    windows: List[TimeWindow] = field(default_factory=list)
    continuous_duration_s: float = 15.0
    light_pattern: str = "WARN_BLINK"
    buzzer_pattern: str = "BEEP"
    cooldown_s: float = 30.0

    def __post_init__(self) -> None:
        if self.event_type not in _VALID_EVENTS:
            raise ValueError(f"invalid event_type={self.event_type!r}")
        # "NONE" passes through unchanged and is interpreted by the dispatcher
        # as "don't touch this channel".
        if self.light_pattern != "NONE" and self.light_pattern not in _LIGHT_PATTERNS:
            raise ValueError(f"invalid light_pattern={self.light_pattern!r}")
        if self.buzzer_pattern != "NONE" and self.buzzer_pattern not in _BUZZER_PATTERNS:
            raise ValueError(f"invalid buzzer_pattern={self.buzzer_pattern!r}")
        if self.continuous_duration_s < 0:
            self.continuous_duration_s = 0.0

    @classmethod
    def from_dict(cls, d: Dict[str, Any]) -> "AlarmRule":
        windows_raw = d.get("windows") or d.get("timeWindows") or []
        windows = [TimeWindow.from_dict(w) for w in windows_raw]
        return cls(
            id=str(d.get("id") or d.get("_id") or ""),
            name=str(d.get("name") or "unnamed"),
            event_type=str(d.get("event_type") or d.get("eventType") or EVENT_ANY_SAFETY).lower(),
            enabled=bool(d.get("enabled", True)),
            windows=windows,
            continuous_duration_s=float(d.get("continuous_duration_s") or d.get("continuousDurationS") or 15.0),
            light_pattern=str(d.get("light_pattern") or d.get("lightPattern") or "WARN_BLINK").upper(),
            buzzer_pattern=str(d.get("buzzer_pattern") or d.get("buzzerPattern") or "BEEP").upper(),
            cooldown_s=float(d.get("cooldown_s") or d.get("cooldownS") or 30.0),
        )

    def active_now(self, now: datetime) -> bool:
        if not self.enabled:
            return False
        if not self.windows:
            return True
        return any(w.contains(now) for w in self.windows)


# Mapping from "any_safety" to the concrete events that satisfy it. Keep
# `schedule` out of this list — schedule rules are time-only and fire on
# their own tick, not on event injection.
_ANY_SAFETY_MEMBERS = {
    EVENT_PERSON, EVENT_FIRE,
    EVENT_BATTERY_LOW, EVENT_BATTERY_CRITICAL,
    EVENT_TIPOVER, EVENT_SYSTEM_ERROR,
}


class AlarmController:
    """In-process rule engine.

    Wiring:
        ctrl = AlarmController(
            send_cmd=motor_controller.send_command,
            publish_trigger=lambda rec: client.publish(T.ALARM_TRIGGERED, json.dumps(rec)),
        )
        # On every safety event:
        ctrl.on_event(EVENT_FIRE, {"confidence": 0.92})
        # Periodically (e.g. once per second) so schedule rules and stale-event
        # self-clear actually fire:
        ctrl.tick()
        # When a new rules payload arrives:
        ctrl.update_rules(payload_list)
    """

    def __init__(
        self,
        send_cmd: Callable[[str], bool],
        publish_trigger: Optional[Callable[[Dict[str, Any]], None]] = None,
        clock: Callable[[], float] = time.monotonic,
        wallclock: Callable[[], datetime] = datetime.now,
        event_idle_clear_s: float = 5.0,
    ):
        self.send_cmd = send_cmd
        self.publish_trigger = publish_trigger
        self._clock = clock
        self._wallclock = wallclock
        self._idle_clear_s = event_idle_clear_s
        self._lock = threading.Lock()
        self._rules: Dict[str, AlarmRule] = {}
        # Per-rule transient state.
        # first_seen: monotonic time of the first matching event in the current
        #             continuous run, or None if no run active.
        # last_seen:  monotonic time of the most recent matching event.
        # last_fired: monotonic time the rule last dispatched actions.
        self._first_seen: Dict[str, float] = {}
        self._last_seen: Dict[str, float] = {}
        self._last_fired: Dict[str, float] = {}
        # Schedule rules fire once per window-entry. Remember the wall-minute
        # we last fired so we don't refire every tick within the same window.
        self._schedule_last_fire_min: Dict[str, int] = {}

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def update_rules(self, raw: Any) -> int:
        """Replace the in-memory rule set from a backend payload.

        `raw` may be a list of dicts (the wire format) or a JSON string. Invalid
        entries are dropped with a warning rather than aborting the whole load.
        Returns the number of rules successfully loaded.
        """
        if isinstance(raw, (bytes, bytearray)):
            try:
                raw = raw.decode("utf-8")
            except UnicodeDecodeError:
                log.warning("[alarm] non-utf8 rules payload, ignored")
                return 0
        if isinstance(raw, str):
            try:
                raw = json.loads(raw)
            except json.JSONDecodeError as exc:
                log.warning("[alarm] bad rules JSON: %s", exc)
                return 0
        if isinstance(raw, dict):
            raw = raw.get("rules") or []
        if not isinstance(raw, list):
            log.warning("[alarm] rules payload must be list, got %s", type(raw).__name__)
            return 0

        loaded: Dict[str, AlarmRule] = {}
        for item in raw:
            try:
                rule = AlarmRule.from_dict(item)
            except Exception as exc:
                log.warning("[alarm] dropping invalid rule %s: %s", item, exc)
                continue
            if not rule.id:
                log.warning("[alarm] dropping rule with missing id: %s", rule.name)
                continue
            loaded[rule.id] = rule

        with self._lock:
            self._rules = loaded
            # Drop transient state for rules that no longer exist.
            for d in (self._first_seen, self._last_seen, self._last_fired, self._schedule_last_fire_min):
                stale = [k for k in d if k not in loaded]
                for k in stale:
                    d.pop(k, None)

        log.info("[alarm] loaded %d rule(s): %s", len(loaded),
                 ", ".join(f"{r.name}/{r.event_type}" for r in loaded.values()))
        return len(loaded)

    def list_rules(self) -> List[Dict[str, Any]]:
        with self._lock:
            return [self._rule_to_dict(r) for r in self._rules.values()]

    def on_event(self, kind: str, meta: Optional[Dict[str, Any]] = None) -> List[str]:
        """Feed one safety event. Returns IDs of rules that fired this call.

        `meta` is forwarded into the `triggered` MQTT record for forensics
        (confidence, percentage, axis, etc.).
        """
        kind = (kind or "").lower().strip()
        if kind not in _VALID_EVENTS or kind == EVENT_SCHEDULE:
            # SCHEDULE never gets injected — it only fires through tick().
            return []

        now_m = self._clock()
        wall = self._wallclock()
        meta = meta or {}
        fired: List[Tuple[AlarmRule, float]] = []

        with self._lock:
            for rule in self._rules.values():
                if not rule.active_now(wall):
                    continue
                if not self._rule_matches(rule, kind):
                    continue

                # Record the matching event. If the previous run went stale
                # (no event for `idle_clear_s`), start a fresh run.
                last = self._last_seen.get(rule.id, 0.0)
                if rule.id not in self._first_seen or (now_m - last) > self._idle_clear_s:
                    self._first_seen[rule.id] = now_m
                self._last_seen[rule.id] = now_m

                dwell = now_m - self._first_seen[rule.id]
                # V5.15c11 (2026-05-26): single-shot safety events fire
                # immediately. tipover_watcher already enforces its own
                # min_dwell_sec (default 0.4s) and only fires ONCE per tip;
                # if we waited for `continuous_duration_s` worth of follow-
                # ups they would never arrive and the rule never triggered.
                # Same logic for battery thresholds — those cross hysteresis
                # exactly once per direction. Streaming detections (person,
                # fire) still honour the dwell so a one-frame YOLO blip
                # cannot wake the buzzer.
                _SINGLE_SHOT_EVENTS = {
                    EVENT_TIPOVER,
                    EVENT_BATTERY_LOW,
                    EVENT_BATTERY_CRITICAL,
                    EVENT_SYSTEM_ERROR,
                }
                if (kind not in _SINGLE_SHOT_EVENTS
                        and dwell < rule.continuous_duration_s):
                    continue

                last_fire = self._last_fired.get(rule.id, 0.0)
                if last_fire and (now_m - last_fire) < rule.cooldown_s:
                    continue

                self._last_fired[rule.id] = now_m
                fired.append((rule, dwell))

        # Dispatch + publish OUTSIDE the lock.
        for rule, dwell in fired:
            self._dispatch(rule, kind, meta, dwell)
        return [r.id for r, _ in fired]

    def tick(self) -> List[str]:
        """Periodic tick — runs schedule rules and resets stale event timers.

        Should be called ~once per second by the host. Returns IDs of schedule
        rules that fired this tick.
        """
        now_m = self._clock()
        wall = self._wallclock()
        cur_minute = wall.hour * 60 + wall.minute
        scheduled: List[Tuple[AlarmRule, float]] = []

        with self._lock:
            # Stale-event sweep — clear continuous-run state when no event for
            # idle_clear_s. Without this, a one-shot person detection would
            # leave the rule "armed" until next reboot and break the dwell
            # semantics.
            stale_ids = [
                rid for rid, last in self._last_seen.items()
                if (now_m - last) > self._idle_clear_s
            ]
            for rid in stale_ids:
                self._first_seen.pop(rid, None)
                self._last_seen.pop(rid, None)

            for rule in self._rules.values():
                if rule.event_type != EVENT_SCHEDULE:
                    continue
                if not rule.active_now(wall):
                    # Once we leave the window, allow refire on next entry.
                    self._schedule_last_fire_min.pop(rule.id, None)
                    continue
                # Only fire once per minute within the active window — the
                # operator-facing semantics are "ring at the start of the
                # window", not "ring continuously until it ends".
                last_min = self._schedule_last_fire_min.get(rule.id)
                if last_min == cur_minute:
                    continue
                self._schedule_last_fire_min[rule.id] = cur_minute
                # Schedule rules ignore continuous_duration_s — they're a
                # time-of-day bell, the time itself is the trigger.
                self._last_fired[rule.id] = now_m
                scheduled.append((rule, 0.0))

        for rule, dwell in scheduled:
            self._dispatch(rule, EVENT_SCHEDULE, {}, dwell)
        return [r.id for r, _ in scheduled]

    # ------------------------------------------------------------------
    # Internals
    # ------------------------------------------------------------------

    @staticmethod
    def _rule_matches(rule: AlarmRule, event_kind: str) -> bool:
        if rule.event_type == event_kind:
            return True
        if rule.event_type == EVENT_ANY_SAFETY and event_kind in _ANY_SAFETY_MEMBERS:
            return True
        return False

    def _dispatch(self, rule: AlarmRule, event_kind: str,
                  meta: Dict[str, Any], dwell: float) -> None:
        """Send LP:/BUZZ: lines + publish a `triggered` record."""
        # Send light then buzzer; failure of one does not block the other.
        if rule.light_pattern and rule.light_pattern != "NONE":
            try:
                self.send_cmd(f"LP:{rule.light_pattern}")
            except Exception as exc:
                log.warning("[alarm] LP send failed for %s: %s", rule.name, exc)
        if rule.buzzer_pattern and rule.buzzer_pattern != "NONE":
            try:
                self.send_cmd(f"BUZZ:{rule.buzzer_pattern}")
            except Exception as exc:
                log.warning("[alarm] BUZZ send failed for %s: %s", rule.name, exc)

        log.warning(
            "[alarm] FIRED rule=%s event=%s dwell=%.1fs light=%s buzzer=%s",
            rule.name, event_kind, dwell, rule.light_pattern, rule.buzzer_pattern,
        )

        if self.publish_trigger is None:
            return
        # Backend (mqtt-ingest) treats this topic as snake_case. Keep keys in
        # snake_case so handleAlarmTriggeredMessage can validate and persist
        # the trigger; otherwise the row is silently dropped at validation.
        record = {
            "rule_id": rule.id,
            "rule_name": rule.name,
            "event_type": event_kind,
            "ts": int(time.time() * 1000),
            "dwell_s": round(dwell, 2),
            "light_pattern": rule.light_pattern,
            "buzzer_pattern": rule.buzzer_pattern,
            "meta": meta,
        }
        try:
            self.publish_trigger(record)
        except Exception as exc:
            log.warning("[alarm] publish_trigger raised: %s", exc)

    @staticmethod
    def _rule_to_dict(rule: AlarmRule) -> Dict[str, Any]:
        return {
            "id": rule.id,
            "name": rule.name,
            "eventType": rule.event_type,
            "enabled": rule.enabled,
            "continuousDurationS": rule.continuous_duration_s,
            "lightPattern": rule.light_pattern,
            "buzzerPattern": rule.buzzer_pattern,
            "cooldownS": rule.cooldown_s,
            "windows": [
                {
                    "startMin": w.start_min,
                    "endMin": w.end_min,
                    "weekdays": sorted(w.weekdays),
                }
                for w in rule.windows
            ],
        }
