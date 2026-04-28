#!/usr/bin/env python3
"""
test_aruco_detection.py — Live ArUco marker detection tester
=============================================================
Fetches JPEG snapshots from the mjpeg_server running on this Pi and
runs ArUco detection on them. Prints detection results continuously
so you can verify markers are recognized and tune placement.

Usage (on Pi):
    python3 test_aruco_detection.py              # default: 2 Hz, DICT_4X4_50
    python3 test_aruco_detection.py --rate 5     # 5 Hz
    python3 test_aruco_detection.py --save       # save annotated frames

Expected setup:
    - mjpeg_server running on port 8080
    - HTTP GET /snapshot returns a JPEG frame
    - Marker dictionary: DICT_4X4_50 (matches generate_markers.py)
    - Marker physical size: 10 cm
"""

import argparse
import os
import sys
import time
import urllib.request
import urllib.error

import cv2
import numpy as np


SNAPSHOT_URL = "http://localhost:8080/snapshot"
MARKER_SIZE_M = 0.10  # 10 cm

# Camera intrinsics for Pi Camera v1 (OV5647) at 960x540 (mjpeg_server stream).
# CALIBRATED empirically: measured 40 cm, original estimate reported 100 cm,
# so fx must be scaled by ~0.40. From fx=820 → fx≈328. This matches a wider
# effective FOV than datasheet nominal, likely because mjpeg_server uses the
# full sensor with binning rather than a center crop.
# For production-grade accuracy, run a proper chessboard calibration.
CAMERA_MATRIX = np.array([
    [328.0,   0.0, 480.0],
    [  0.0, 328.0, 270.0],
    [  0.0,   0.0,   1.0],
], dtype=np.float32)
DIST_COEFFS = np.zeros((5,), dtype=np.float32)

# Marker ID labels (match generate_markers.py)
MARKER_LABELS = {
    0: "HOME",
    1: "CORNER 1",
    2: "CORNER 2",
    3: "CORNER 3",
    4: "CORNER 4",
}

# Only accept markers in this set (filter out false positives from background)
VALID_IDS = set(MARKER_LABELS.keys())

# Reject pose estimates farther than this (likely noise / bad pose)
MAX_ACCEPT_DISTANCE_M = 4.0


def fetch_snapshot(url: str, timeout: float = 1.0) -> np.ndarray:
    """Fetch JPEG from URL and decode to BGR ndarray. Returns None on error."""
    try:
        with urllib.request.urlopen(url, timeout=timeout) as resp:
            data = resp.read()
    except (urllib.error.URLError, TimeoutError) as e:
        print(f"  [error] fetch: {e}")
        return None

    arr = np.frombuffer(data, dtype=np.uint8)
    frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    if frame is None:
        print("  [error] decode failed")
    return frame


def build_detector():
    """Build an ArUco detector for DICT_4X4_50."""
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    params = cv2.aruco.DetectorParameters()
    # Tuning for print-on-paper markers under indoor lighting
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 23
    params.adaptiveThreshWinSizeStep = 10
    params.minMarkerPerimeterRate = 0.03
    params.maxMarkerPerimeterRate = 4.0
    params.polygonalApproxAccuracyRate = 0.05
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.cornerRefinementWinSize = 5
    params.cornerRefinementMaxIterations = 30
    return cv2.aruco.ArucoDetector(aruco_dict, params)


def estimate_pose(corners_list):
    """Estimate 6-DoF pose for each detected marker.

    Returns list of (rvec, tvec) tuples.

    Handles both OpenCV 4.x (estimatePoseSingleMarkers) and the newer API
    that uses solvePnP with marker object points.
    """
    results = []
    # Object points for a single marker at origin
    half = MARKER_SIZE_M / 2.0
    obj_pts = np.array([
        [-half,  half, 0.0],
        [ half,  half, 0.0],
        [ half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float32)

    for corners in corners_list:
        img_pts = corners.reshape((4, 2)).astype(np.float32)
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts, CAMERA_MATRIX, DIST_COEFFS,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if ok:
            results.append((rvec, tvec))
        else:
            results.append((None, None))
    return results


def describe_pose(tvec):
    """Convert tvec to (distance, forward, left, bearing_deg) tuple.
    Camera frame: x=right, y=down, z=forward
    Robot frame (top-down): x=forward, y=left"""
    import math
    flat = np.asarray(tvec).reshape(-1)
    tx, ty, tz = float(flat[0]), float(flat[1]), float(flat[2])
    rel_forward = tz
    rel_left = -tx
    distance = (tx * tx + tz * tz) ** 0.5
    bearing_deg = math.degrees(math.atan2(rel_left, rel_forward)) if rel_forward > 0.01 else 0.0
    return distance, rel_forward, rel_left, bearing_deg


def format_pose(distance, rel_forward, rel_left, bearing_deg) -> str:
    return (
        f"dist={distance:.2f}m  "
        f"fwd={rel_forward:+.2f}m  "
        f"left={rel_left:+.2f}m  "
        f"bearing={bearing_deg:+.1f}°"
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rate", type=float, default=2.0, help="detection rate Hz")
    ap.add_argument("--url", default=SNAPSHOT_URL, help="snapshot URL")
    ap.add_argument("--save", action="store_true", help="save annotated frames")
    ap.add_argument("--save-dir", default="/tmp/aruco_debug", help="save directory")
    ap.add_argument("--duration", type=float, default=0, help="stop after N seconds (0=forever)")
    args = ap.parse_args()

    if args.save:
        os.makedirs(args.save_dir, exist_ok=True)
        print(f"Saving annotated frames to {args.save_dir}")

    print("=" * 70)
    print("K-Patrol ArUco Detection Live Test")
    print("=" * 70)
    print(f"  Snapshot URL: {args.url}")
    print(f"  Rate:         {args.rate} Hz")
    print(f"  Dictionary:   DICT_4X4_50")
    print(f"  Marker size:  {MARKER_SIZE_M*100:.0f} cm")
    print(f"  Camera:       Pi Camera v1 (OV5647), 960x540")
    print()
    print("Press Ctrl+C to stop")
    print("-" * 70)

    detector = build_detector()
    interval = 1.0 / args.rate
    frame_count = 0
    detected_frames = 0
    start_time = time.time()
    last_detected_ids = set()

    try:
        while True:
            t0 = time.time()
            if args.duration > 0 and t0 - start_time > args.duration:
                break
            frame = fetch_snapshot(args.url)
            if frame is None:
                time.sleep(interval)
                continue
            frame_count += 1

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = detector.detectMarkers(gray)

            accepted = []       # list of (marker_id, pose_tuple, corners_idx)
            rejected_info = []  # list of (marker_id, reason)

            if ids is not None and len(ids) > 0:
                poses = estimate_pose(corners)
                for i, marker_id_arr in enumerate(ids.flatten()):
                    marker_id = int(marker_id_arr)
                    rvec, tvec = poses[i]

                    # Filter 1: only accept valid IDs
                    if marker_id not in VALID_IDS:
                        rejected_info.append((marker_id, "invalid_id"))
                        continue

                    # Filter 2: pose estimation succeeded
                    if tvec is None:
                        rejected_info.append((marker_id, "pose_fail"))
                        continue

                    # Filter 3: reasonable distance
                    distance, rel_fwd, rel_left, bearing = describe_pose(tvec)
                    if distance > MAX_ACCEPT_DISTANCE_M:
                        rejected_info.append((marker_id, f"too_far_{distance:.1f}m"))
                        continue

                    accepted.append((marker_id, (distance, rel_fwd, rel_left, bearing), i))

            if accepted:
                detected_frames += 1
                current_ids = set(m[0] for m in accepted)

                ts = time.strftime("%H:%M:%S")
                print(f"[{ts}] Frame {frame_count} — {len(accepted)} marker(s)")
                for marker_id, pose_tuple, corners_idx in accepted:
                    label = MARKER_LABELS.get(marker_id, f"ID{marker_id}")
                    # Pixel edge length (for calibration sanity check)
                    c = corners[corners_idx].reshape(4, 2)
                    edges_px = [
                        ((c[0] - c[1])**2).sum()**0.5,
                        ((c[1] - c[2])**2).sum()**0.5,
                        ((c[2] - c[3])**2).sum()**0.5,
                        ((c[3] - c[0])**2).sum()**0.5,
                    ]
                    avg_edge_px = sum(edges_px) / 4
                    print(f"   ID{marker_id:>2} ({label:<9}) {format_pose(*pose_tuple)}  edge={avg_edge_px:.0f}px")

                if rejected_info:
                    rej_str = ", ".join(f"ID{rid}({reason})" for rid, reason in rejected_info)
                    print(f"   [skip] {rej_str}")

                new_ids = current_ids - last_detected_ids
                lost_ids = last_detected_ids - current_ids
                if new_ids:
                    print(f"   [+] New markers: {sorted(new_ids)}")
                if lost_ids:
                    print(f"   [-] Lost markers: {sorted(lost_ids)}")
                last_detected_ids = current_ids

                if args.save:
                    annotated = cv2.aruco.drawDetectedMarkers(frame.copy(), corners, ids)
                    for i, (rvec, tvec) in enumerate(poses):
                        if rvec is not None:
                            cv2.drawFrameAxes(annotated, CAMERA_MATRIX, DIST_COEFFS,
                                              rvec, tvec, MARKER_SIZE_M * 0.5)
                    fname = os.path.join(args.save_dir, f"frame_{frame_count:05d}.jpg")
                    cv2.imwrite(fname, annotated)
            else:
                if last_detected_ids:
                    print(f"[{time.strftime('%H:%M:%S')}] Lost all markers")
                last_detected_ids = set()
                if rejected_info:
                    rej_str = ", ".join(f"ID{rid}({reason})" for rid, reason in rejected_info)
                    print(f"[{time.strftime('%H:%M:%S')}] Rejected: {rej_str}")

            elapsed = time.time() - t0
            sleep_time = max(0, interval - elapsed)
            time.sleep(sleep_time)

    except KeyboardInterrupt:
        pass

    print()
    print("-" * 70)
    total = time.time() - start_time
    print(f"Frames fetched: {frame_count}")
    print(f"Frames with markers: {detected_frames} ({100 * detected_frames / max(1, frame_count):.1f}%)")
    print(f"Average FPS: {frame_count / max(0.01, total):.1f}")


if __name__ == "__main__":
    main()
