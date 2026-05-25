#!/usr/bin/env python3
"""
test_avoid_fsm.py — verify obstacle-avoidance sub-FSM in LineFollower.

Injects a mock front-distance provider that toggles between "clear" and
"obstacle close" at controlled instants, drives the controller with a
synthetic straight-line frame, and asserts the 3-phase detour pattern:

    STRAFE (left/right) → FORWARD → STRAFE_BACK → NONE (line re-acquired)

Exit code 0 on pass, 1 on failure. Run from `robots/pi-controller`:

    python3 tools/test_avoid_fsm.py
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import cv2
import numpy as np

# Allow `from navigation import ...` when launched from tools/.
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from navigation.line_follower import LineFollower, LineFollowerConfig  # noqa: E402


def _straight_line_frame(h: int = 480, w: int = 640) -> np.ndarray:
    """Synthetic dark floor with a vertical white tape stripe centred."""
    frame = np.full((h, w, 3), 30, dtype=np.uint8)  # dark grey floor
    cv2.rectangle(frame, (w // 2 - 12, 0), (w // 2 + 12, h),
                  (240, 240, 240), thickness=-1)
    return frame


def main() -> int:
    cfg = LineFollowerConfig()
    cfg.avoid_strafe_duration_s      = 0.5
    cfg.avoid_forward_duration_s     = 0.8
    cfg.avoid_strafe_back_duration_s = 0.5

    lf = LineFollower(cfg)

    # Mock front-distance — flips between "clear" and "obstacle close" via
    # a closure cell so the test can mutate it across ticks.
    state = {"d_mm": 1500.0}

    def provider() -> float:
        return state["d_mm"]

    lf.set_front_distance_provider(provider)
    frame = _straight_line_frame()

    log: list[tuple[float, str, int, int, int]] = []
    t_start = time.monotonic()

    def step(label: str) -> None:
        r = lf.tick(frame, produce_overlay=False)
        log.append((time.monotonic() - t_start, r.avoid_phase or "-",
                    r.vx, r.vy, r.wz))

    # ── Phase 0 — no obstacle, run a few ticks (vision pipeline should run).
    for _ in range(3):
        step("clear")
        time.sleep(0.05)

    # ── Trigger obstacle.
    state["d_mm"] = 180.0  # below tof_emergency_stop_mm=250
    step("obstacle-enter")
    # FSM must be in STRAFE phase now.
    assert log[-1][1] == "STRAFE", f"expected STRAFE on entry, got {log[-1]}"
    assert log[-1][0] == log[-1][0]  # noqa

    # ── Walk through STRAFE phase (0.5s wall time).
    end_t = time.monotonic() + 0.6
    while time.monotonic() < end_t:
        step("strafe")
        time.sleep(0.02)
        if log[-1][1] != "STRAFE":
            break
    # By now we should be in FORWARD.
    assert log[-1][1] == "FORWARD", f"expected FORWARD after STRAFE, got {log[-1]}"

    # ── Walk through FORWARD (0.8s). Simulate the robot driving past the
    #    obstacle midway through this phase by clearing the front distance.
    end_t = time.monotonic() + 0.85
    cleared = False
    while time.monotonic() < end_t:
        step("forward")
        time.sleep(0.02)
        if not cleared and (time.monotonic() - t_start) > 1.0:
            state["d_mm"] = 1500.0  # obstacle now behind robot
            cleared = True
        if log[-1][1] != "FORWARD":
            break
    assert log[-1][1] == "STRAFE_BACK", \
        f"expected STRAFE_BACK after FORWARD, got {log[-1]}"

    # ── Walk through STRAFE_BACK (0.5s). Obstacle should remain clear.
    end_t = time.monotonic() + 0.6
    while time.monotonic() < end_t:
        step("strafe_back")
        time.sleep(0.02)
        if log[-1][1] != "STRAFE_BACK":
            break
    # FSM should have exited — last tick is either NONE (FSM exit branch) or
    # empty (vision pipeline took over). Either is acceptable.
    last = log[-1]
    assert last[1] in ("", "NONE", "-"), \
        f"expected exit to vision pipeline, got {last}"

    # ── Phase summary
    phases = [row[1] for row in log]
    phase_counts: dict[str, int] = {}
    for p in phases:
        phase_counts[p] = phase_counts.get(p, 0) + 1

    print("─── avoid FSM trace ───────────────────────────────────")
    for row in log:
        t, ph, vx, vy, wz = row
        print(f"  t={t:5.2f}s phase={ph:<11s}  vx={vx:+4d}  vy={vy:+4d}  wz={wz:+4d}")
    print("─── phase counts ──────────────────────────────────────")
    for ph, n in phase_counts.items():
        print(f"  {ph or '(vision)':<12s} : {n} ticks")

    # All phases must have appeared.
    for required in ("STRAFE", "FORWARD", "STRAFE_BACK"):
        assert phase_counts.get(required, 0) > 0, \
            f"phase {required} never observed"

    print("\n✅ Obstacle-avoidance FSM works: STRAFE → FORWARD → STRAFE_BACK → vision")
    return 0


if __name__ == "__main__":
    sys.exit(main())
