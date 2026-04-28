"""
script_patrol.py — Scripted Patrol Executor for K-Patrol Mecanum
================================================================
Deterministic, IMU-closed-loop patrol by executing a declarative script
of primitive motions. No mapping, no odometry, no line/marker detection.

Transport
---------
The executor produces BOTH a legacy single-letter motor command
(F/B/L/R/S/SL/SR) and, when MEC mode is enabled, an explicit twist
(vx, vy, wz, spd) that maps directly to the ESP32-S3 `MEC:` command.
Yaw-hold closed-loop correction is injected on translational primitives
to keep the robot driving straight regardless of motor asymmetry.

Script primitives
-----------------
    rotate          angle_deg + direction       → IMU closed-loop turn
    rotate_to       heading_deg                 → IMU closed-loop turn to absolute heading (relative to script-start yaw)
    forward_time    duration_s                  → open-loop forward (yaw-held)
    backward_time   duration_s                  → open-loop reverse  (yaw-held)
    strafe_time     duration_s + direction      → Mecanum sideways   (yaw-held)
    forward_until   tof_min_cm (+ sensor)       → drive until obstacle
    strafe_until    direction + tof_min_cm      → strafe until obstacle
    move_distance   distance_m + direction      → drive a metric distance (yaw-held)
    arc             distance_m + angle_deg + direction → combined vx+wz
    path            waypoints: [{x,y},...]      → expanded to rotate+move_distance at load time
    pause           duration_s                  → stop, let AI pipeline work

Script JSON format
------------------
    {
        "name": "lobby_rectangle",
        "loop": true,
        "max_iterations": null,
        "default_speed_pct": 60,
        "steps": [
            {"op": "forward_time",  "duration_s": 4.0},
            {"op": "rotate",        "angle_deg": 90, "direction": "left"},
            {"op": "move_distance", "distance_m": 1.5, "direction": "forward"},
            {"op": "path",          "waypoints": [{"x":0,"y":0},{"x":1,"y":0},{"x":1,"y":1}]},
            ...
        ]
    }

Safety
------
During any motion primitive, emergency stop fires if the relevant ToF
sensor reads below config.emergency_tof_mm. Rotation is in-place and
does not trigger forward-cone checks.
"""

from __future__ import annotations

import json
import math
import os
import time
from dataclasses import asdict, dataclass, field
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple


# ── ESP32-S3 motor command strings ─────────────────────────────────────────
CMD_FWD      = "F"
CMD_BACK     = "B"
CMD_LEFT     = "L"
CMD_RIGHT    = "R"
CMD_STOP     = "S"
CMD_STRAFE_L = "SL"
CMD_STRAFE_R = "SR"


# ── Primitive type enum ────────────────────────────────────────────────────
class PrimitiveType(str, Enum):
    ROTATE         = "rotate"
    ROTATE_TO      = "rotate_to"
    FORWARD_TIME   = "forward_time"
    BACKWARD_TIME  = "backward_time"
    STRAFE_TIME    = "strafe_time"
    FORWARD_UNTIL  = "forward_until"
    STRAFE_UNTIL   = "strafe_until"
    MOVE_DISTANCE  = "move_distance"
    ARC            = "arc"
    PATH           = "path"
    PAUSE          = "pause"


# ── Executor state ─────────────────────────────────────────────────────────
class ExecutorState(str, Enum):
    IDLE      = "IDLE"
    RUNNING   = "RUNNING"
    COMPLETE  = "COMPLETE"
    EMERGENCY = "EMERGENCY"
    ERROR     = "ERROR"


# ── Configuration ──────────────────────────────────────────────────────────
@dataclass
class ScriptConfig:
    # Rotation closed-loop
    rotate_tol_deg:      float = 2.0
    rotate_timeout_s:    float = 15.0
    rotate_max_tick_deg: float = 30.0
    # Two-phase rotation: decelerate in the last `rotate_slowdown_deg` to
    # absorb momentum + command latency so the robot stops on target.
    rotate_slowdown_deg: float = 12.0
    rotate_slow_pwm:     int   = 80

    # Safety — emergency stop if any relevant ToF reads below this
    emergency_tof_mm:    float = 180.0

    # Hard upper bound on any single step (prevents runaway on bad script)
    max_step_duration_s: float = 60.0

    # Motor ramp compensation (s). Motors take ~100ms to reach target from
    # rest; extend timed primitives so the commanded duration matches real
    # travel time.
    motor_ramp_s:        float = 0.10

    # Speed mapping: pct (0-100) → PWM (0-255)
    min_pwm:             int = 60
    max_pwm:             int = 255
    default_speed_pct:   int = 60

    # ── MEC twist output ──
    # When True, tick() also emits a (vx, vy, wz, spd) twist mapping to
    # the firmware `MEC:vx,vy,wz,spd` command for per-wheel control.
    use_mec:             bool  = True

    # Yaw-hold PID (P-only) correction applied on translational primitives.
    # Units: wz counts per degree of drift (wz ∈ [-127, 127]).
    yaw_hold_kp:             float = 2.0
    yaw_hold_max_wz:         int   = 60
    yaw_hold_deadband_deg:   float = 0.5
    # If IMU yaw convention has CCW = positive (ROS-style), set True.
    # If CCW = negative, set False — the controller flips the correction sign.
    yaw_left_is_positive:    bool  = True

    # Calibration — max physical speed at 100% PWM. Used by move_distance / arc
    # to convert distance ↔ time and angle ↔ yaw-rate. These must be measured
    # and tuned on the target robot for accurate distance execution.
    full_linear_mps:         float = 0.50   # m/s at max_pwm, no yaw correction
    full_angular_dps:        float = 90.0   # deg/s at max_pwm during rotate

    # Move-distance two-phase: decelerate in the last `move_slowdown_m` metres
    # at `move_slow_speed_frac` of commanded speed to cut overshoot.
    move_slowdown_m:         float = 0.10
    move_slow_speed_frac:    float = 0.40

    # Encoder-based closed-loop for move_distance.
    # Encoder: 11 PPR × 34:1 gearbox × 4 (quadrature) = 1496 counts per output-shaft rev.
    # Wheel: 97 mm mecanum roller diameter → circumference ≈ 0.305 m.
    # Thus counts_per_m ≈ 1496 / 0.305 ≈ 4905. Measured on the bench.
    encoder_counts_per_m:    float = 4905.0
    # When True and encoder data is supplied to tick(), move_distance closes
    # the loop on wheel counts instead of integrating time × calibrated speed.
    encoder_closed_loop:     bool  = True
    # Tolerance for early completion (m). Encoder quantisation alone is
    # ~0.0002 m, so 5 mm is well above the noise floor and below human-visible
    # precision for a 1-m patrol leg.
    encoder_tol_m:           float = 0.005

    # Arc primitive — minimum radius to prevent degenerate in-place arcs.
    arc_min_radius_m:        float = 0.05

    # Path compiler
    path_min_segment_m:      float = 0.02   # drop segments shorter than this
    path_simplify_epsilon_m: float = 0.03   # RDP simplification epsilon

    def pct_to_pwm(self, pct: int) -> int:
        pct = max(0, min(100, int(pct)))
        if pct == 0:
            return 0
        return max(self.min_pwm, int(round(self.max_pwm * pct / 100)))

    def linear_mps_for(self, pct: int) -> float:
        pct = max(0, min(100, int(pct)))
        return self.full_linear_mps * (pct / 100.0)


# ── Script data model ──────────────────────────────────────────────────────
@dataclass
class ScriptStep:
    op: str
    angle_deg:   Optional[float] = None
    direction:   Optional[str]   = None   # "left" | "right" | "forward" | "backward"
    duration_s:  Optional[float] = None
    speed_pct:   Optional[int]   = None   # overrides script default
    tof_min_cm:  Optional[float] = None
    tof_sensor:  Optional[str]   = None
    # New fields for metric + absolute-heading primitives
    distance_m:  Optional[float] = None
    heading_deg: Optional[float] = None
    radius_m:    Optional[float] = None
    waypoints:   Optional[List[Dict[str, float]]] = None

    def to_dict(self) -> Dict[str, Any]:
        return {k: v for k, v in asdict(self).items() if v is not None}


@dataclass
class PatrolScript:
    name:               str
    steps:              List[ScriptStep] = field(default_factory=list)
    loop:               bool = True
    max_iterations:     Optional[int] = None
    default_speed_pct:  int = 60

    def to_dict(self) -> Dict[str, Any]:
        return {
            "name": self.name,
            "loop": self.loop,
            "max_iterations": self.max_iterations,
            "default_speed_pct": self.default_speed_pct,
            "steps": [s.to_dict() for s in self.steps],
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "PatrolScript":
        steps = [ScriptStep(**s) for s in data.get("steps", [])]
        return cls(
            name               = str(data["name"]),
            steps              = steps,
            loop               = bool(data.get("loop", True)),
            max_iterations     = data.get("max_iterations"),
            default_speed_pct  = int(data.get("default_speed_pct", 60)),
        )


# ── Validation ─────────────────────────────────────────────────────────────
def validate_step(step: ScriptStep) -> Optional[str]:
    """Return None if valid, else an error message."""
    try:
        op = PrimitiveType(step.op)
    except ValueError:
        return f"unknown op '{step.op}'"

    if op == PrimitiveType.ROTATE:
        if step.angle_deg is None or step.angle_deg <= 0 or step.angle_deg > 180:
            return "rotate requires 0 < angle_deg <= 180"
        if step.direction not in ("left", "right"):
            return "rotate requires direction='left' or 'right'"

    elif op == PrimitiveType.ROTATE_TO:
        if step.heading_deg is None:
            return "rotate_to requires heading_deg"

    elif op in (PrimitiveType.FORWARD_TIME, PrimitiveType.BACKWARD_TIME, PrimitiveType.PAUSE):
        if step.duration_s is None or step.duration_s <= 0:
            return f"{op.value} requires duration_s > 0"

    elif op == PrimitiveType.STRAFE_TIME:
        if step.duration_s is None or step.duration_s <= 0:
            return "strafe_time requires duration_s > 0"
        if step.direction not in ("left", "right"):
            return "strafe_time requires direction='left' or 'right'"

    elif op == PrimitiveType.FORWARD_UNTIL:
        if step.tof_min_cm is None or step.tof_min_cm <= 0:
            return "forward_until requires tof_min_cm > 0"

    elif op == PrimitiveType.STRAFE_UNTIL:
        if step.tof_min_cm is None or step.tof_min_cm <= 0:
            return "strafe_until requires tof_min_cm > 0"
        if step.direction not in ("left", "right"):
            return "strafe_until requires direction='left' or 'right'"

    elif op == PrimitiveType.MOVE_DISTANCE:
        if step.distance_m is None or step.distance_m <= 0:
            return "move_distance requires distance_m > 0"
        if step.direction not in ("forward", "backward", "left", "right"):
            return "move_distance requires direction in {forward,backward,left,right}"

    elif op == PrimitiveType.ARC:
        if step.distance_m is None or step.distance_m <= 0:
            return "arc requires distance_m > 0"
        if step.angle_deg is None or step.angle_deg == 0:
            return "arc requires angle_deg != 0"
        if step.direction not in ("left", "right"):
            return "arc requires direction='left' or 'right'"

    elif op == PrimitiveType.PATH:
        if not step.waypoints or len(step.waypoints) < 2:
            return "path requires at least 2 waypoints"
        for i, wp in enumerate(step.waypoints):
            if not isinstance(wp, dict) or "x" not in wp or "y" not in wp:
                return f"path waypoint[{i}] requires x and y"

    if step.speed_pct is not None and not (0 <= step.speed_pct <= 100):
        return "speed_pct must be in [0, 100]"

    return None


def validate_script(script: PatrolScript) -> List[str]:
    errors: List[str] = []
    if not script.name:
        errors.append("script name is required")
    if not script.steps:
        errors.append("script has no steps")
    for idx, step in enumerate(script.steps):
        err = validate_step(step)
        if err:
            errors.append(f"step[{idx}]: {err}")
    if script.max_iterations is not None and script.max_iterations <= 0:
        errors.append("max_iterations must be > 0 or null")
    return errors


# ── Tick result ────────────────────────────────────────────────────────────
@dataclass
class TickResult:
    motor_cmd: str                                   # legacy F/B/L/R/S/SL/SR
    speed_pwm: Optional[int]                         # None = unchanged
    status:    Dict[str, Any]
    # MEC twist (vx, vy, wz, spd); None if MEC disabled or STOP state.
    twist:     Optional[Tuple[int, int, int, int]] = None


# ── Angle / path math ──────────────────────────────────────────────────────
def signed_angle_diff(a: float, b: float) -> float:
    """Shortest signed difference a - b, wrapped to (-180, 180]."""
    return (a - b + 180.0) % 360.0 - 180.0


def _rdp_simplify(pts: List[Tuple[float, float]], epsilon: float) -> List[Tuple[float, float]]:
    """Ramer-Douglas-Peucker polyline simplification."""
    if len(pts) < 3:
        return list(pts)

    def _perp_dist(p, a, b):
        ax, ay = a
        bx, by = b
        px, py = p
        dx, dy = bx - ax, by - ay
        if dx == 0 and dy == 0:
            return math.hypot(px - ax, py - ay)
        t = ((px - ax) * dx + (py - ay) * dy) / (dx * dx + dy * dy)
        t = max(0.0, min(1.0, t))
        cx, cy = ax + t * dx, ay + t * dy
        return math.hypot(px - cx, py - cy)

    max_d = 0.0
    index = 0
    for i in range(1, len(pts) - 1):
        d = _perp_dist(pts[i], pts[0], pts[-1])
        if d > max_d:
            max_d = d
            index = i
    if max_d > epsilon:
        left  = _rdp_simplify(pts[: index + 1], epsilon)
        right = _rdp_simplify(pts[index:], epsilon)
        return left[:-1] + right
    return [pts[0], pts[-1]]


def compile_path_to_steps(
    waypoints: List[Dict[str, float]],
    cfg: ScriptConfig,
    speed_pct: Optional[int] = None,
) -> List[ScriptStep]:
    """Compile a list of (x, y) waypoints → rotate + move_distance steps.

    Assumes the robot starts at waypoints[0] facing +x (path-local heading 0°).
    Strategy: turn-in-place then straight-line to each successive point.
    Simplifies the polyline with RDP first so fine-grained drawn paths don't
    explode into hundreds of micro-steps.
    """
    if not waypoints or len(waypoints) < 2:
        return []

    pts = [(float(wp["x"]), float(wp["y"])) for wp in waypoints]
    if cfg.path_simplify_epsilon_m > 0:
        pts = _rdp_simplify(pts, cfg.path_simplify_epsilon_m)

    steps: List[ScriptStep] = []
    current_heading = 0.0
    prev = pts[0]
    for pt in pts[1:]:
        dx = pt[0] - prev[0]
        dy = pt[1] - prev[1]
        dist = math.hypot(dx, dy)
        if dist < cfg.path_min_segment_m:
            continue
        target_heading = math.degrees(math.atan2(dy, dx))
        delta = signed_angle_diff(target_heading, current_heading)
        if abs(delta) > cfg.rotate_tol_deg:
            if delta > 0:
                steps.append(ScriptStep(
                    op="rotate", angle_deg=abs(delta), direction="left",
                    speed_pct=speed_pct,
                ))
            else:
                steps.append(ScriptStep(
                    op="rotate", angle_deg=abs(delta), direction="right",
                    speed_pct=speed_pct,
                ))
        steps.append(ScriptStep(
            op="move_distance", distance_m=dist, direction="forward",
            speed_pct=speed_pct,
        ))
        current_heading = target_heading
        prev = pt

    return steps


# ── Executor ───────────────────────────────────────────────────────────────
class ScriptExecutor:
    """Tick-driven executor. One call per control loop tick."""

    def __init__(self, config: Optional[ScriptConfig] = None):
        self.config = config or ScriptConfig()
        self.script: Optional[PatrolScript] = None

        self.state: ExecutorState = ExecutorState.IDLE
        self._step_idx: int = 0
        self._iteration: int = 0

        self._step_start_time: Optional[float] = None
        self._step_start_yaw:  Optional[float] = None
        self._script_start_yaw: Optional[float] = None
        self._last_speed_pwm:  Optional[int]   = None
        self._last_twist:      Optional[Tuple[int,int,int,int]] = None
        self._error:           Optional[str]   = None

        # Rotate accumulator — sums per-tick |Δyaw| to avoid wrap ambiguity
        # at exactly ±180°, which causes runaway rotations with a naive
        # abs(signed_angle_diff(yaw, start_yaw)) comparison.
        self._rot_accum_deg:   float           = 0.0
        self._rot_prev_yaw:    Optional[float] = None
        # For rotate_to: the target yaw in IMU frame (absolute) + direction.
        self._rot_target_yaw:  Optional[float] = None
        self._rot_direction:   Optional[str]   = None
        # Arc primitive side-channel: _dispatch_arc stashes a wz value here
        # and _to_twist replaces the default CMD_FWD yaw-hold correction
        # with this curvature command. Cleared on step advance.
        self._arc_wz_override: Optional[int]   = None

        # Encoder-closed-loop move_distance anchors. Populated on the first
        # tick of each move_distance step and reset on advance. We track
        # the average of all four wheels (absolute counts) so drift on any
        # single wheel doesn't short-circuit the leg.
        self._enc_start_counts: Optional[Dict[str, int]] = None
        self._enc_last_distance_m: float = 0.0

    # ----- public API -----
    def load(self, script: PatrolScript) -> List[str]:
        """Load a script. Expands PATH steps. Returns validation errors."""
        errors = validate_script(script)
        if errors:
            self.state = ExecutorState.ERROR
            self._error = "; ".join(errors)
            self.script = None
            return errors
        # Expand PATH steps into rotate + move_distance sequence.
        self.script = self._expand_path_steps(script)
        self.state = ExecutorState.IDLE
        self._reset_run_state()
        self._error = None
        return []

    def start(self) -> bool:
        if self.script is None:
            self.state = ExecutorState.ERROR
            self._error = "no_script_loaded"
            return False
        self.state = ExecutorState.RUNNING
        self._reset_run_state()
        return True

    def stop(self) -> None:
        """Abort execution; stays loaded."""
        self.state = ExecutorState.IDLE
        self._step_start_time = None
        self._step_start_yaw = None

    def clear_emergency(self) -> bool:
        """Recover from EMERGENCY to IDLE so operator can restart."""
        if self.state == ExecutorState.EMERGENCY:
            self.state = ExecutorState.IDLE
            self._error = None
            return True
        return False

    def is_running(self) -> bool:
        return self.state == ExecutorState.RUNNING

    def status(self) -> Dict[str, Any]:
        return self._status()

    # ----- core tick -----
    def tick(
        self,
        imu_yaw_deg: float,
        tof_data: Dict[str, float],
        now: Optional[float] = None,
        encoder_counts: Optional[Dict[str, int]] = None,
    ) -> TickResult:
        if now is None:
            now = time.monotonic()

        if self.state != ExecutorState.RUNNING:
            return self._result(CMD_STOP, None, yaw=imu_yaw_deg, apply_yaw_hold=False)

        # Capture script-start yaw once so rotate_to can operate in a
        # frame relative to the initial heading (makes scripts portable).
        if self._script_start_yaw is None:
            self._script_start_yaw = imu_yaw_deg

        step = self._current_step()
        if step is None:
            # End of steps — loop or finish
            if self._should_loop():
                self._iteration += 1
                self._step_idx = 0
                self._advance_step()  # also clears rotate accumulators
                step = self._current_step()
            else:
                self.state = ExecutorState.COMPLETE
                return self._result(CMD_STOP, None, yaw=imu_yaw_deg, apply_yaw_hold=False)

        # Safety: any relevant ToF dangerously close → EMERGENCY
        danger_sensor = self._check_emergency(step, tof_data)
        if danger_sensor is not None:
            self.state = ExecutorState.EMERGENCY
            self._error = f"emergency_{danger_sensor}"
            return self._result(CMD_STOP, None, yaw=imu_yaw_deg, apply_yaw_hold=False)

        # Initialise step anchors on first tick of this step
        if self._step_start_time is None:
            self._step_start_time = now
            self._step_start_yaw = imu_yaw_deg

        elapsed = now - self._step_start_time
        if elapsed > self.config.max_step_duration_s:
            # Runaway protection — advance with stop
            self._advance_step()
            return self._result(CMD_STOP, None, yaw=imu_yaw_deg, apply_yaw_hold=False)

        cmd, done, speed_pwm = self._dispatch(
            step, imu_yaw_deg, tof_data, elapsed, encoder_counts,
        )
        if done:
            self._advance_step()
            return self._result(CMD_STOP, None, yaw=imu_yaw_deg, apply_yaw_hold=False)

        apply_yaw_hold = self._is_yaw_held_primitive(step.op)
        return self._result(cmd, speed_pwm, yaw=imu_yaw_deg, apply_yaw_hold=apply_yaw_hold)

    # ----- internals -----
    def _reset_run_state(self) -> None:
        self._step_idx = 0
        self._iteration = 0
        self._step_start_time = None
        self._step_start_yaw = None
        self._script_start_yaw = None
        self._last_speed_pwm = None
        self._last_twist = None
        self._error = None
        self._rot_accum_deg = 0.0
        self._rot_prev_yaw = None
        self._rot_target_yaw = None
        self._rot_direction = None
        self._arc_wz_override = None
        self._enc_start_counts = None
        self._enc_last_distance_m = 0.0

    def _current_step(self) -> Optional[ScriptStep]:
        if self.script is None or self._step_idx >= len(self.script.steps):
            return None
        return self.script.steps[self._step_idx]

    def _advance_step(self) -> None:
        self._step_idx += 1
        self._step_start_time = None
        self._step_start_yaw = None
        self._rot_accum_deg = 0.0
        self._rot_prev_yaw = None
        self._rot_target_yaw = None
        self._rot_direction = None
        self._arc_wz_override = None
        self._enc_start_counts = None
        self._enc_last_distance_m = 0.0

    def _should_loop(self) -> bool:
        if not self.script or not self.script.loop:
            return False
        if self.script.max_iterations is None:
            return True
        return (self._iteration + 1) < self.script.max_iterations

    def _speed_for(self, step: ScriptStep) -> int:
        pct = step.speed_pct if step.speed_pct is not None else (
            self.script.default_speed_pct if self.script else self.config.default_speed_pct
        )
        return self.config.pct_to_pwm(pct)

    def _speed_pct_for(self, step: ScriptStep) -> int:
        pct = step.speed_pct if step.speed_pct is not None else (
            self.script.default_speed_pct if self.script else self.config.default_speed_pct
        )
        return max(0, min(100, int(pct)))

    def _expand_path_steps(self, script: PatrolScript) -> PatrolScript:
        expanded: List[ScriptStep] = []
        for step in script.steps:
            if step.op == PrimitiveType.PATH.value and step.waypoints:
                expanded.extend(compile_path_to_steps(
                    step.waypoints, self.config, speed_pct=step.speed_pct,
                ))
            else:
                expanded.append(step)
        script.steps = expanded
        return script

    @staticmethod
    def _is_yaw_held_primitive(op: str) -> bool:
        return op in (
            PrimitiveType.FORWARD_TIME.value,
            PrimitiveType.BACKWARD_TIME.value,
            PrimitiveType.FORWARD_UNTIL.value,
            PrimitiveType.STRAFE_TIME.value,
            PrimitiveType.STRAFE_UNTIL.value,
            PrimitiveType.MOVE_DISTANCE.value,
        )

    def _result(
        self,
        cmd: str,
        speed_pwm: Optional[int],
        yaw: float,
        apply_yaw_hold: bool,
    ) -> TickResult:
        # Deduplicate SPD:<n> commands — only emit on change.
        emit_speed: Optional[int] = None
        if speed_pwm is not None and speed_pwm != self._last_speed_pwm:
            emit_speed = speed_pwm
            self._last_speed_pwm = speed_pwm

        twist: Optional[Tuple[int, int, int, int]] = None
        if self.config.use_mec:
            twist = self._to_twist(cmd, speed_pwm, yaw, apply_yaw_hold)
            # Deduplicate twists too so we don't flood the serial link.
            if twist == self._last_twist:
                pass  # Still return; nav loop dedupes at emit level.
            self._last_twist = twist

        return TickResult(
            motor_cmd=cmd,
            speed_pwm=emit_speed,
            status=self._status(),
            twist=twist,
        )

    def _status(self) -> Dict[str, Any]:
        total = len(self.script.steps) if self.script else 0
        current_op = None
        if self.script and 0 <= self._step_idx < total:
            current_op = self.script.steps[self._step_idx].op
        return {
            "state":       self.state.value,
            "script":      self.script.name if self.script else None,
            "step_idx":    self._step_idx,
            "total_steps": total,
            "current_op":  current_op,
            "iteration":   self._iteration,
            "error":       self._error,
        }

    # ----- twist mapping -----
    def _to_twist(
        self,
        cmd: str,
        speed_pwm: Optional[int],
        yaw: float,
        apply_yaw_hold: bool,
    ) -> Tuple[int, int, int, int]:
        """Map legacy motor_cmd + speed to (vx, vy, wz, spd) for MEC."""
        if cmd == CMD_STOP:
            return (0, 0, 0, 0)

        spd = int(speed_pwm) if speed_pwm is not None else self.config.max_pwm
        spd = max(0, min(255, spd))

        wz_corr = 0
        if apply_yaw_hold and self._step_start_yaw is not None:
            wz_corr = self._yaw_correction_wz(yaw, self._step_start_yaw)

        if cmd == CMD_FWD:
            # Arc primitive overrides the yaw-hold correction with an
            # explicit curvature command (vx + wz combined).
            if self._arc_wz_override is not None:
                return (127, 0, int(self._arc_wz_override), spd)
            return (127, 0, wz_corr, spd)
        if cmd == CMD_BACK:
            return (-127, 0, wz_corr, spd)
        if cmd == CMD_LEFT:
            # In-place rotate CCW — no yaw-hold (we're commanding yaw).
            return (0, 0, 127, spd)
        if cmd == CMD_RIGHT:
            return (0, 0, -127, spd)
        if cmd == CMD_STRAFE_L:
            # +vy = left (ROS convention). Yaw-hold keeps heading steady.
            return (0, 127, wz_corr, spd)
        if cmd == CMD_STRAFE_R:
            return (0, -127, wz_corr, spd)
        return (0, 0, 0, 0)

    def _yaw_correction_wz(self, yaw: float, target_yaw: float) -> int:
        """P-only correction to hold heading during translation."""
        drift = signed_angle_diff(yaw, target_yaw)
        if abs(drift) < self.config.yaw_hold_deadband_deg:
            return 0
        sign = 1 if self.config.yaw_left_is_positive else -1
        # When drift > 0 (current yaw CCW of target), rotate CW → wz negative
        # in ROS convention. Multiply by -sign to get the right polarity.
        wz = -sign * self.config.yaw_hold_kp * drift
        max_wz = self.config.yaw_hold_max_wz
        return int(max(-max_wz, min(max_wz, wz)))

    # ----- per-primitive dispatch -----
    def _dispatch(
        self,
        step: ScriptStep,
        yaw:  float,
        tof:  Dict[str, float],
        elapsed: float,
        encoder_counts: Optional[Dict[str, int]] = None,
    ) -> Tuple[str, bool, Optional[int]]:
        """Return (motor_cmd, done, speed_pwm)."""
        op = step.op

        if op == PrimitiveType.PAUSE.value:
            return CMD_STOP, elapsed >= (step.duration_s or 0.0), None

        speed_pwm = self._speed_for(step)

        # Motor-ramp compensation for timed primitives.
        timed_target = (step.duration_s or 0.0) + self.config.motor_ramp_s

        if op == PrimitiveType.FORWARD_TIME.value:
            return CMD_FWD, elapsed >= timed_target, speed_pwm

        if op == PrimitiveType.BACKWARD_TIME.value:
            return CMD_BACK, elapsed >= timed_target, speed_pwm

        if op == PrimitiveType.STRAFE_TIME.value:
            cmd = CMD_STRAFE_L if step.direction == "left" else CMD_STRAFE_R
            return cmd, elapsed >= timed_target, speed_pwm

        if op == PrimitiveType.FORWARD_UNTIL.value:
            sensor = step.tof_sensor or "front"
            reading = tof.get(sensor, 9999.0)
            min_mm = (step.tof_min_cm or 0.0) * 10.0
            if 0 < reading <= min_mm:
                return CMD_STOP, True, None
            return CMD_FWD, False, speed_pwm

        if op == PrimitiveType.STRAFE_UNTIL.value:
            direction = step.direction or "left"
            sensor = step.tof_sensor or direction
            reading = tof.get(sensor, 9999.0)
            min_mm = (step.tof_min_cm or 0.0) * 10.0
            cmd = CMD_STRAFE_L if direction == "left" else CMD_STRAFE_R
            if 0 < reading <= min_mm:
                return CMD_STOP, True, None
            return cmd, False, speed_pwm

        if op == PrimitiveType.ROTATE.value:
            return self._dispatch_rotate(step, yaw, elapsed, speed_pwm)

        if op == PrimitiveType.ROTATE_TO.value:
            return self._dispatch_rotate_to(step, yaw, elapsed, speed_pwm)

        if op == PrimitiveType.MOVE_DISTANCE.value:
            return self._dispatch_move_distance(
                step, yaw, elapsed, speed_pwm, encoder_counts,
            )

        if op == PrimitiveType.ARC.value:
            return self._dispatch_arc(step, yaw, elapsed, speed_pwm)

        # Unknown op — should have been filtered by validation
        return CMD_STOP, True, None

    def _dispatch_rotate(
        self,
        step: ScriptStep,
        yaw:  float,
        elapsed: float,
        speed_pwm: int,
    ) -> Tuple[str, bool, Optional[int]]:
        """
        Accumulate per-tick |Δyaw| until it reaches the target angle.

        Why not abs(signed_angle_diff(yaw, start_yaw))?
        signed_angle_diff wraps to (-180, 180]; at a 180° target the
        measurement is ambiguous at the turnaround and the robot routinely
        missed the stop and overshot to 360° / 540°.
        Summing small per-tick deltas avoids the wrap entirely.
        """
        target = abs(step.angle_deg or 0.0)
        cmd = CMD_LEFT if step.direction == "left" else CMD_RIGHT

        if self._rot_prev_yaw is None:
            self._rot_prev_yaw = yaw
        else:
            tick_delta = abs(signed_angle_diff(yaw, self._rot_prev_yaw))
            if tick_delta < self.config.rotate_max_tick_deg:
                self._rot_accum_deg += tick_delta
            self._rot_prev_yaw = yaw

        remaining = target - self._rot_accum_deg

        if remaining <= self.config.rotate_tol_deg:
            return CMD_STOP, True, None

        if elapsed > self.config.rotate_timeout_s:
            self._error = (
                f"rotate_timeout delta={self._rot_accum_deg:.1f}/{target:.1f}"
            )
            return CMD_STOP, True, None

        if remaining <= self.config.rotate_slowdown_deg:
            effective_pwm = min(speed_pwm, self.config.rotate_slow_pwm)
        else:
            effective_pwm = speed_pwm

        return cmd, False, effective_pwm

    def _dispatch_rotate_to(
        self,
        step: ScriptStep,
        yaw:  float,
        elapsed: float,
        speed_pwm: int,
    ) -> Tuple[str, bool, Optional[int]]:
        """
        Turn to an absolute heading (relative to script-start yaw). On the
        first tick of this step we compute the shortest signed delta and
        remember the direction, then fall through to the same accumulator
        logic as plain rotate.
        """
        if self._rot_target_yaw is None:
            # Heading is in script-local frame: absolute yaw = start + heading.
            ref = self._script_start_yaw if self._script_start_yaw is not None else yaw
            target_yaw = ref + float(step.heading_deg or 0.0)
            delta = signed_angle_diff(target_yaw, yaw)
            self._rot_target_yaw = target_yaw
            self._rot_direction = "left" if delta > 0 else "right"
            # Store |delta| as the effective target so we can reuse the
            # accumulator path below.
            # Stash on the step object (transient) so _dispatch_rotate sees it.
            step.angle_deg = abs(delta)
            step.direction = self._rot_direction

        # Shortcut — already on target.
        if self._rot_target_yaw is not None:
            err = abs(signed_angle_diff(yaw, self._rot_target_yaw))
            if err <= self.config.rotate_tol_deg and self._rot_accum_deg > 0:
                return CMD_STOP, True, None

        return self._dispatch_rotate(step, yaw, elapsed, speed_pwm)

    def _dispatch_move_distance(
        self,
        step: ScriptStep,
        yaw:  float,
        elapsed: float,
        speed_pwm: int,
        encoder_counts: Optional[Dict[str, int]] = None,
    ) -> Tuple[str, bool, Optional[int]]:
        """
        Metric translation with yaw-hold.

        Closed loop when encoders are available (default): terminate the leg
        when the mean wheel travel reaches `distance_m` (within `encoder_tol_m`).
        Falls back to the original open-loop timing path when encoders are
        unavailable or disabled — preserves behaviour on benches that don't
        expose encoder telemetry.
        """
        dist = float(step.distance_m or 0.0)
        pct = self._speed_pct_for(step)
        mps = self.config.linear_mps_for(pct)
        if mps <= 0 or dist <= 0:
            return CMD_STOP, True, None

        direction = step.direction or "forward"
        if direction == "forward":
            cmd = CMD_FWD
        elif direction == "backward":
            cmd = CMD_BACK
        elif direction == "left":
            cmd = CMD_STRAFE_L
        elif direction == "right":
            cmd = CMD_STRAFE_R
        else:
            return CMD_STOP, True, None

        use_enc = (
            self.config.encoder_closed_loop
            and self.config.encoder_counts_per_m > 0
            and encoder_counts is not None
            and any(k in encoder_counts for k in ("FR", "FL", "BR", "BL"))
        )

        # Time-based fallback parameters — also used as a failsafe upper
        # bound when running closed-loop so a slipping wheel or a missing
        # encoder message can't pin a step forever.
        total_time = dist / mps + self.config.motor_ramp_s
        # Closed-loop failsafe: allow 2× the nominal travel time before
        # giving up on this step (compare to 1× for pure open-loop).
        failsafe_time = (total_time * 2.0) if use_enc else total_time
        slow_dist = min(self.config.move_slowdown_m, dist * 0.5)

        if use_enc:
            # Anchor the starting counts on first tick. Strafing exercises
            # all four wheels in opposing directions, so we work with the
            # absolute delta per wheel (|cnt - start|) and average them —
            # that gives a robust distance estimate regardless of direction.
            if self._enc_start_counts is None:
                self._enc_start_counts = {
                    k: int(encoder_counts.get(k, 0))
                    for k in ("FR", "FL", "BR", "BL")
                }

            deltas: List[float] = []
            for k in ("FR", "FL", "BR", "BL"):
                if k in encoder_counts and k in self._enc_start_counts:
                    deltas.append(abs(
                        int(encoder_counts[k]) - int(self._enc_start_counts[k])
                    ))
            # Mean count delta → metric distance via calibrated counts/m.
            if deltas:
                mean_counts = sum(deltas) / len(deltas)
                travelled = mean_counts / self.config.encoder_counts_per_m
            else:
                travelled = 0.0
            self._enc_last_distance_m = travelled

            remaining = dist - travelled
            # Early termination on target.
            if remaining <= self.config.encoder_tol_m:
                return CMD_STOP, True, None
            # Failsafe: if we've spent way more than expected time without
            # hitting target, call it done rather than hanging forever.
            if elapsed >= failsafe_time:
                self._error = (
                    f"move_distance_timeout travelled={travelled:.3f}/"
                    f"{dist:.3f} t={elapsed:.1f}s"
                )
                return CMD_STOP, True, None
            # Two-phase deceleration based on actual remaining distance.
            if remaining <= slow_dist:
                slow_pwm = max(
                    self.config.min_pwm,
                    int(speed_pwm * self.config.move_slow_speed_frac),
                )
                return cmd, False, slow_pwm
            return cmd, False, speed_pwm

        # ── open-loop fallback (encoders unavailable) ──
        slow_time = slow_dist / max(mps, 1e-6)
        if elapsed >= total_time:
            return CMD_STOP, True, None
        if elapsed >= (total_time - slow_time):
            slow_pwm = max(
                self.config.min_pwm,
                int(speed_pwm * self.config.move_slow_speed_frac),
            )
            return cmd, False, slow_pwm
        return cmd, False, speed_pwm

    def _dispatch_arc(
        self,
        step: ScriptStep,
        yaw:  float,
        elapsed: float,
        speed_pwm: int,
    ) -> Tuple[str, bool, Optional[int]]:
        """
        Combined forward + rotate. Computed open-loop: time from arc length
        and linear speed, yaw-rate from angle/time. Emitted as legacy F
        command; the actual curvature is set in the MEC twist by the nav
        loop if arc state is exposed (currently approximate).

        Note: the legacy single-letter channel can't express combined vx+wz,
        so for non-MEC transports this degenerates into straight forward.
        MEC mode gets proper arc behaviour via the twist override below.
        """
        dist = float(step.distance_m or 0.0)
        angle = float(step.angle_deg or 0.0)
        pct = self._speed_pct_for(step)
        mps = self.config.linear_mps_for(pct)
        if mps <= 0 or dist <= 0:
            return CMD_STOP, True, None

        total_time = dist / mps + self.config.motor_ramp_s
        if elapsed >= total_time:
            return CMD_STOP, True, None

        # Stash arc parameters so _to_twist can override wz for this step.
        # Simpler: compute wz here and bypass yaw-hold for ARC by setting
        # direction (CMD_FWD with a dedicated twist override).
        # We use a side-channel attribute on self so _to_twist picks it up.
        yaw_rate_dps = angle / max(total_time, 1e-6)
        if step.direction == "right":
            yaw_rate_dps = -abs(yaw_rate_dps)
        else:
            yaw_rate_dps = abs(yaw_rate_dps)
        # Map degrees/s → wz counts: full_angular_dps corresponds to |wz|=127.
        if self.config.full_angular_dps > 0:
            wz = int(max(-127, min(127, 127 * yaw_rate_dps / self.config.full_angular_dps)))
        else:
            wz = 0
        self._arc_wz_override = wz

        return CMD_FWD, False, speed_pwm

    # ----- safety: emergency ToF -----
    def _check_emergency(
        self,
        step: ScriptStep,
        tof: Dict[str, float],
    ) -> Optional[str]:
        """Return offending sensor name, or None."""
        sensors = self._emergency_sensors(step)
        for name in sensors:
            reading = tof.get(name)
            if reading is None:
                continue
            if 0 < reading < self.config.emergency_tof_mm:
                return name
        return None

    @staticmethod
    def _emergency_sensors(step: ScriptStep) -> Tuple[str, ...]:
        op = step.op
        if op in (
            PrimitiveType.FORWARD_TIME.value,
            PrimitiveType.FORWARD_UNTIL.value,
            PrimitiveType.ARC.value,
        ):
            return ("front", "front_left", "front_right")
        if op == PrimitiveType.BACKWARD_TIME.value:
            return ("back",)
        if op in (PrimitiveType.STRAFE_TIME.value, PrimitiveType.STRAFE_UNTIL.value):
            return (step.direction or "left",)
        if op == PrimitiveType.MOVE_DISTANCE.value:
            direction = step.direction or "forward"
            if direction == "forward":
                return ("front", "front_left", "front_right")
            if direction == "backward":
                return ("back",)
            return (direction,)
        # rotate, rotate_to, pause → no translational emergency
        return ()


# ── Script I/O ─────────────────────────────────────────────────────────────
class ScriptLibrary:
    """Filesystem-backed collection of patrol scripts (one JSON per script)."""

    def __init__(self, directory: str):
        self.directory = directory
        os.makedirs(self.directory, exist_ok=True)

    def _path(self, name: str) -> str:
        if not name or not all(c.isalnum() or c in ("-", "_") for c in name):
            raise ValueError(f"invalid script name: {name!r}")
        return os.path.join(self.directory, f"{name}.json")

    def list_scripts(self) -> List[str]:
        if not os.path.isdir(self.directory):
            return []
        names = []
        for fname in sorted(os.listdir(self.directory)):
            if fname.endswith(".json"):
                names.append(fname[:-5])
        return names

    def load(self, name: str) -> PatrolScript:
        path = self._path(name)
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        if "name" not in data:
            data["name"] = name
        return PatrolScript.from_dict(data)

    def save(self, script: PatrolScript) -> str:
        errors = validate_script(script)
        if errors:
            raise ValueError("invalid script: " + "; ".join(errors))
        path = self._path(script.name)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(script.to_dict(), f, indent=2, ensure_ascii=False)
        return path

    def delete(self, name: str) -> bool:
        path = self._path(name)
        if os.path.exists(path):
            os.remove(path)
            return True
        return False


# ── Example scripts (for reference / default library) ──────────────────────
def example_rectangle(side1_s: float = 4.0, side2_s: float = 2.5) -> PatrolScript:
    """Rectangular patrol: 4 forward segments + 4 left turns, loop forever."""
    steps = []
    durations = [side1_s, side2_s, side1_s, side2_s]
    for d in durations:
        steps.append(ScriptStep(op="forward_time", duration_s=d))
        steps.append(ScriptStep(op="pause", duration_s=1.5))
        steps.append(ScriptStep(op="rotate", angle_deg=90, direction="left"))
    return PatrolScript(
        name="rectangle",
        steps=steps,
        loop=True,
        default_speed_pct=60,
    )


__all__ = [
    "CMD_FWD", "CMD_BACK", "CMD_LEFT", "CMD_RIGHT",
    "CMD_STOP", "CMD_STRAFE_L", "CMD_STRAFE_R",
    "PrimitiveType", "ExecutorState",
    "ScriptConfig", "ScriptStep", "PatrolScript",
    "validate_step", "validate_script",
    "TickResult", "signed_angle_diff",
    "compile_path_to_steps",
    "ScriptExecutor", "ScriptLibrary",
    "example_rectangle",
]
