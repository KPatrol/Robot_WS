"""
reactive_nav.py — Behavior-based reactive navigation middleware.

Sits between any nav mode (script patrol, line follow, free coverage) and the
ESP32 UART interface. Intercepts (vx, vy, wz, spd) on every tick, reads ToF
readings, and applies a priority behavior stack to modify the output.

Coordinate system (matches firmware MEC: IK):
    vx  +forward / -backward
    vy  +strafe-right / -strafe-left
    wz  +rotate-left(CCW) / -rotate-right(CW)
    spd  absolute magnitude 0-255

Priority stack (highest first):
  P1 EMERGENCY  front < emergency_mm      → full stop + reverse pulse
  P2 BRAKE      front < brake_mm          → scale speed, reduce vx
  P3 STEER      front_left < steer_mm     → vy+ wz- (veer right)
                front_right < steer_mm    → vy- wz+ (veer left)
  P4 SIDE       left < side_mm            → vy+ (strafe away)
                right < side_mm           → vy- (strafe away)
  P5 CLEAR      pass-through unchanged
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field
from enum import Enum
from typing import NamedTuple, Optional


# ── Configurable thresholds ────────────────────────────────────────────────────

@dataclass
class ReactiveConfig:
    emergency_mm:  int   = 150   # hard stop + back-pulse
    brake_mm:      int   = 450   # progressive speed ramp-down starts here
    brake_min_mm:  int   = 150   # speed at this distance → min_speed_factor
    min_speed_fac: float = 0.20  # minimum forward speed factor when braking
    steer_mm:      int   = 300   # front-diagonal sensors trigger steering
    side_mm:       int   = 180   # side sensors trigger vy strafe
    back_mm:       int   = 120   # rear sensor prevents backing into obstacle

    steer_vy:      int   = 40    # vy correction magnitude for steer
    steer_wz:      int   = 25    # wz correction magnitude for steer
    side_vy:       int   = 35    # vy correction magnitude for side avoidance

    back_pulse_spd: int  = 40    # reverse speed after emergency (brief pulse)
    back_pulse_ms:  int  = 300   # duration of reverse pulse (ms)


# ── Behavior enum ──────────────────────────────────────────────────────────────

class Behavior(str, Enum):
    EMERGENCY  = "EMERGENCY"
    BRAKE      = "BRAKE"
    STEER_R    = "STEER_R"   # veering right (obstacle on left-front)
    STEER_L    = "STEER_L"   # veering left  (obstacle on right-front)
    SIDE_R     = "SIDE_R"    # strafe right  (obstacle on left side)
    SIDE_L     = "SIDE_L"    # strafe left   (obstacle on right side)
    CLEAR      = "CLEAR"


# ── Output tuple ──────────────────────────────────────────────────────────────

class ReactiveResult(NamedTuple):
    vx:       int
    vy:       int
    wz:       int
    spd:      int
    behavior: Behavior
    tof_min:  int   # closest reading (for telemetry)


# ── Simple ToF snapshot (decoupled from sensor_reader import) ─────────────────

@dataclass
class ToFSnapshot:
    front:       int = 9999
    front_left:  int = 9999
    front_right: int = 9999
    left:        int = 9999
    right:       int = 9999
    back:        int = 9999

    @classmethod
    def from_sensor(cls, tof) -> "ToFSnapshot":
        """Accept a sensor_reader.ToFData or any object with same attributes."""
        return cls(
            front=int(getattr(tof, "front", 9999)),
            front_left=int(getattr(tof, "front_left", 9999)),
            front_right=int(getattr(tof, "front_right", 9999)),
            left=int(getattr(tof, "left", 9999)),
            right=int(getattr(tof, "right", 9999)),
            back=int(getattr(tof, "back", 9999)),
        )

    def min_distance(self) -> int:
        return min(
            self.front, self.front_left, self.front_right,
            self.left, self.right, self.back,
        )


# ── Reactive navigator ─────────────────────────────────────────────────────────

class ReactiveNav:
    """
    Stateful behavior middleware.

    Usage:
        rnav = ReactiveNav()
        result = rnav.process(tof_snapshot, vx=80, vy=0, wz=0, spd=60)
        # result.vx, result.vy, result.wz, result.spd, result.behavior
    """

    def __init__(self, config: Optional[ReactiveConfig] = None):
        self.cfg = config or ReactiveConfig()
        self._in_emergency: bool = False
        self._emergency_until: float = 0.0   # wall-clock time to hold back pulse

    def process(
        self,
        tof: ToFSnapshot,
        vx: int = 0,
        vy: int = 0,
        wz: int = 0,
        spd: int = 60,
    ) -> ReactiveResult:
        cfg = self.cfg
        tof_min = tof.min_distance()

        # ── P1: Emergency ──────────────────────────────────────────────────────
        if tof.front <= cfg.emergency_mm:
            self._in_emergency = True
            self._emergency_until = time.monotonic() + cfg.back_pulse_ms / 1000.0

        if self._in_emergency:
            if time.monotonic() < self._emergency_until:
                # Brief reverse pulse — ignore any side/back block (already stopped)
                rvx = -80 if tof.back > cfg.back_mm else 0
                return ReactiveResult(rvx, 0, 0, cfg.back_pulse_spd, Behavior.EMERGENCY, tof_min)
            else:
                self._in_emergency = False

        # ── P2: Progressive braking on approaching front obstacle ─────────────
        if tof.front < cfg.brake_mm and vx > 0:
            rng = max(1, cfg.brake_mm - cfg.brake_min_mm)
            progress = 1.0 - max(0.0, min(1.0, (tof.front - cfg.brake_min_mm) / rng))
            speed_fac = 1.0 - (1.0 - cfg.min_speed_fac) * progress
            spd = max(int(spd * speed_fac), int(spd * cfg.min_speed_fac))
            vx = int(vx * speed_fac)
            behavior = Behavior.BRAKE
        else:
            behavior = Behavior.CLEAR

        # ── P3: Diagonal steering ─────────────────────────────────────────────
        # Both triggered → pick the closer obstacle
        hit_l = tof.front_left < cfg.steer_mm
        hit_r = tof.front_right < cfg.steer_mm

        if hit_l and hit_r:
            # Both blocked — steer toward the wider side
            hit_l = tof.front_left <= tof.front_right
            hit_r = not hit_l

        if hit_l:
            # Obstacle on front-left → veer right: vy+ wz-
            strength = max(0.0, 1.0 - tof.front_left / cfg.steer_mm)
            vy = int(vy + cfg.steer_vy * strength)
            wz = int(wz - cfg.steer_wz * strength)
            behavior = Behavior.STEER_R
        elif hit_r:
            # Obstacle on front-right → veer left: vy- wz+
            strength = max(0.0, 1.0 - tof.front_right / cfg.steer_mm)
            vy = int(vy - cfg.steer_vy * strength)
            wz = int(wz + cfg.steer_wz * strength)
            behavior = Behavior.STEER_L

        # ── P4: Side avoidance (strafe only, don't override P3 behavior label) ─
        if tof.left < cfg.side_mm:
            strength = max(0.0, 1.0 - tof.left / cfg.side_mm)
            vy = int(vy + cfg.side_vy * strength)
            if behavior == Behavior.CLEAR:
                behavior = Behavior.SIDE_R

        if tof.right < cfg.side_mm:
            strength = max(0.0, 1.0 - tof.right / cfg.side_mm)
            vy = int(vy - cfg.side_vy * strength)
            if behavior == Behavior.CLEAR:
                behavior = Behavior.SIDE_L

        # ── Back safety: suppress reverse if backing into obstacle ───────────
        if vx < 0 and tof.back < cfg.back_mm:
            vx = 0

        # ── Clamp to valid range ──────────────────────────────────────────────
        vx  = max(-127, min(127, vx))
        vy  = max(-127, min(127, vy))
        wz  = max(-127, min(127, wz))
        spd = max(0,    min(255, spd))

        return ReactiveResult(vx, vy, wz, spd, behavior, tof_min)

    def reset(self) -> None:
        self._in_emergency = False
        self._emergency_until = 0.0


__all__ = ["ReactiveNav", "ReactiveConfig", "ReactiveResult", "Behavior", "ToFSnapshot"]
