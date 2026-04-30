"""Closed-loop driver: FreeCoverage + Odometry + RobotSim + World.

Validates the full nav stack offline. Replaces the v5 main loop with a
pure-Python tick at 20 Hz against the kinematic simulator: the FSM reads
synthetic ToFs from the world, the odometer fuses synthetic encoder counts
+ noisy IMU yaw, and the resulting body twist is fed back into the robot.

Run:
    cd robots/pi-controller
    python -m sim.run_free_coverage              # 60 s headless run
    python -m sim.run_free_coverage --duration 30 --verbose

Reports:
    - Final coverage_pct from the OccupancyGrid
    - Total collisions
    - Final pose, state distribution
"""

from __future__ import annotations

import argparse
import math
import time
from collections import Counter

import navigation.free_coverage as _fc_module
from navigation.free_coverage import CoverageConfig, FreeCoverage
from navigation.odometry import Odometry

from .mecanum_world import RobotSim, WorldConfig, make_demo_world


class _SimClock:
    """Stand-in for the `time` module so FreeCoverage's state timers tick
    against simulated time, not wall-clock time. Without this, a 60-s sim
    that runs in 0.3 s of wall time leaves the FSM permanently stuck in
    its first TURNING state because `time.monotonic()` barely advances."""

    def __init__(self) -> None:
        self.t = 0.0

    def monotonic(self) -> float:
        return self.t


# Mirror the v5 conversion: FreeCoverage emits normalized vx/vy/wz in
# [-127, +127] alongside an absolute speed in [0, 255]. The Pi converts
# these into physical body twist using the operator-tunable max_mps /
# max_wz envelope.
MEC_NORM_MAX  = 127.0
SPD_NORM_MAX  = 255.0
VPID_MAX_MPS  = 1.0   # KPATROL_VPID_MAX_MPS default in v5
VPID_MAX_WZ   = 3.0   # KPATROL_VPID_MAX_WZ default in v5


def normalized_to_body_twist(vx_n: int, vy_n: int, wz_n: int, spd: int):
    """Convert FreeCoverage's MEC-normalized output to physical body twist."""
    scale = (spd / SPD_NORM_MAX) if spd else 0.0
    vx_mps = (vx_n / MEC_NORM_MAX) * scale * VPID_MAX_MPS
    vy_mps = (vy_n / MEC_NORM_MAX) * scale * VPID_MAX_MPS
    wz_rps = (wz_n / MEC_NORM_MAX) * scale * VPID_MAX_WZ
    return vx_mps, vy_mps, wz_rps


def run_simulation(
    duration_s: float = 60.0,
    dt: float = 0.05,
    verbose: bool = False,
    seed: int = 42,
    encoder_noise: float = 0.0,
    imu_noise_deg: float = 0.0,
) -> dict:
    world_cfg = WorldConfig(
        encoder_noise=encoder_noise,
        imu_noise_deg=imu_noise_deg,
    )
    world = make_demo_world(world_cfg)
    robot = RobotSim(world, x=0.0, y=0.0, theta_rad=0.0, ramp_tau_s=0.05)
    odo = Odometry(imu_alpha=0.05)
    fc = FreeCoverage(CoverageConfig())

    # Inject sim-time clock into FreeCoverage so its state-duration timers
    # (turn dwell, frontier-recheck period) advance with sim_t, not wall time.
    clock = _SimClock()
    real_time = _fc_module.time
    _fc_module.time = clock

    try:
        # Anchor odometry with the initial (zero) encoder counts before the
        # first motion tick — matches v5's bootstrap.
        odo.update(robot.encoder_counts, imu_yaw_deg=robot.imu_yaw_deg(),
                   now=clock.monotonic())

        n_ticks = int(round(duration_s / dt))
        state_counts: Counter = Counter()
        sim_t = 0.0
        last_log = 0.0

        for k in range(n_ticks):
            sim_t = k * dt
            clock.t = sim_t
            tofs = robot.tofs_mm()
            pose = odo.update(
                robot.encoder_counts,
                imu_yaw_deg=robot.imu_yaw_deg(),
                now=clock.monotonic(),
            )

            result = fc.tick(
                front_mm=tofs["front"],
                left_mm=tofs["left"],
                right_mm=tofs["right"],
                pose=pose,
                all_tofs=tofs,
            )
            state_counts[result.state] += 1

            vx_mps, vy_mps, wz_rps = normalized_to_body_twist(
                result.vx, result.vy, result.wz, result.spd
            )
            robot.step(vx_mps, vy_mps, wz_rps, dt)

            if verbose and sim_t - last_log >= 5.0:
                diag = fc.diagnostics()
                grid = diag.get("map", {})
                print(
                    f"  t={sim_t:5.1f}s  state={result.state:<13s} "
                    f"pose=({pose.x:+.2f},{pose.y:+.2f},θ={math.degrees(pose.theta):+.1f}°) "
                    f"coverage={grid.get('coverage_pct', 0.0):.1f}%  "
                    f"collisions={robot.collisions}"
                )
                last_log = sim_t

        diag = fc.diagnostics()
        grid = diag.get("map", {})
        final_pose = odo.get_pose()
        return {
            "duration_s":   duration_s,
            "ticks":        n_ticks,
            "coverage_pct": float(grid.get("coverage_pct", 0.0)),
            "visited":      int(grid.get("visited", 0)),
            "obstacles":    int(grid.get("obstacles", 0)),
            "collisions":   robot.collisions,
            "final_pose":   (final_pose.x, final_pose.y, final_pose.theta),
            "state_dist":   dict(state_counts),
        }
    finally:
        _fc_module.time = real_time


def main() -> None:
    parser = argparse.ArgumentParser(description="Headless FreeCoverage simulator")
    parser.add_argument("--duration", type=float, default=60.0,
                        help="simulated seconds (default 60)")
    parser.add_argument("--dt", type=float, default=0.05,
                        help="simulation step (default 0.05 s = 20 Hz)")
    parser.add_argument("--verbose", action="store_true",
                        help="print state every 5 sim-seconds")
    parser.add_argument("--seed", type=int, default=42)
    parser.add_argument("--encoder-noise", type=float, default=0.0,
                        help="std dev of per-tick encoder count noise")
    parser.add_argument("--imu-noise", type=float, default=0.0,
                        help="std dev of yaw noise (degrees)")
    args = parser.parse_args()

    print(f"[sim] FreeCoverage smoke-run for {args.duration:.0f} s "
          f"(dt={args.dt:.3f}, demo world)")
    t0 = time.monotonic()
    summary = run_simulation(
        args.duration, args.dt, args.verbose, args.seed,
        encoder_noise=args.encoder_noise,
        imu_noise_deg=args.imu_noise,
    )
    t1 = time.monotonic()

    print()
    print(f"[sim] Wall time:        {t1 - t0:.2f} s "
          f"(real-time factor ≈ {args.duration / max(1e-6, t1 - t0):.1f}×)")
    print(f"[sim] Ticks:            {summary['ticks']}")
    print(f"[sim] Coverage:         {summary['coverage_pct']:.2f} % "
          f"(visited={summary['visited']}, obstacles={summary['obstacles']})")
    print(f"[sim] Collisions:       {summary['collisions']}")
    fp = summary["final_pose"]
    print(f"[sim] Final pose:       x={fp[0]:+.2f}m  y={fp[1]:+.2f}m  "
          f"θ={math.degrees(fp[2]):+.1f}°")
    print(f"[sim] State histogram:  {summary['state_dist']}")


if __name__ == "__main__":
    main()
