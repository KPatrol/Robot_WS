"""
Evaluate the K-Patrol detector on the curated dataset and produce the thesis
figures: ROC curve, PR curve, confusion matrix, and a model-comparison table.

Two detectors are compared:
  - `yolov8n.pt` (nano, default on the Pi)
  - `yolov8s.pt` (small, heavier — run on a laptop to establish the upper bound)

For fire we don't have a trained model; instead we sweep `fire_min_area_ratio`
over the HSV segmenter and report the same curves, so the thresholds chapter
can be written with real numbers.

Usage:
    python -m detection.eval_models \\
        --datasets detection/datasets \\
        --out reports/ \\
        --models yolov8n.pt yolov8s.pt

Outputs under `--out/`:
    roc_person.png  pr_person.png  confusion_person.png
    roc_fire.png    pr_fire.png    confusion_fire.png
    model_comparison.csv
    thresholds.json

Dependencies: numpy, matplotlib, scikit-learn, ultralytics, opencv-python.
All imports are lazy so the module can be unit-tested without the heavy deps.
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

log = logging.getLogger("kpatrol.eval")


# ---------------------------------------------------------------------------
# Dataset loading
# ---------------------------------------------------------------------------

@dataclass
class Sample:
    relpath: str
    label_person: int
    label_fire: int
    note: str


def load_manifest(datasets_dir: Path) -> List[Sample]:
    manifest = datasets_dir / "manifest.csv"
    if not manifest.exists():
        raise FileNotFoundError(
            f"Manifest not found: {manifest}. See datasets/README.md for layout."
        )
    out: List[Sample] = []
    with manifest.open() as f:
        for row in csv.DictReader(f):
            out.append(
                Sample(
                    relpath=row["relpath"],
                    label_person=int(row["label_person"]),
                    label_fire=int(row["label_fire"]),
                    note=row.get("note", ""),
                )
            )
    log.info("[eval] loaded %d samples from %s", len(out), manifest)
    return out


# ---------------------------------------------------------------------------
# Scorers
# ---------------------------------------------------------------------------

def score_person_yolo(
    samples: List[Sample],
    datasets_dir: Path,
    model_name: str,
    imgsz: int = 320,
) -> Tuple[List[int], List[float], float]:
    """Run YOLO over every image; return (y_true, y_score, avg_fps).

    `y_score` is the max confidence of any `person` box in the frame (0.0 if
    no box fires). Missing images are skipped with a warning.
    """
    from ultralytics import YOLO  # type: ignore
    import cv2  # type: ignore

    model = YOLO(model_name)
    y_true: List[int] = []
    y_score: List[float] = []
    total_ms = 0.0
    n = 0

    for s in samples:
        img_path = datasets_dir / s.relpath
        if not img_path.exists():
            log.warning("[eval] missing image: %s", img_path)
            continue
        img = cv2.imread(str(img_path))
        if img is None:
            continue

        t0 = time.perf_counter()
        results = model.predict(img, imgsz=imgsz, classes=[0], conf=0.05, verbose=False)
        total_ms += (time.perf_counter() - t0) * 1000
        n += 1

        max_conf = 0.0
        for r in results:
            boxes = getattr(r, "boxes", None)
            if boxes is None:
                continue
            for b in boxes:
                max_conf = max(max_conf, float(b.conf[0]))

        y_true.append(s.label_person)
        y_score.append(max_conf)

    avg_fps = (1000.0 / (total_ms / n)) if n else 0.0
    log.info("[eval] %s: %d frames, avg %.1f ms/frame, %.1f FPS",
             model_name, n, total_ms / max(n, 1), avg_fps)
    return y_true, y_score, avg_fps


def score_fire_hsv(
    samples: List[Sample],
    datasets_dir: Path,
) -> Tuple[List[int], List[float], float]:
    """HSV segmenter: score = red-mask area ratio. Returns (y_true, y_score, fps)."""
    import cv2  # type: ignore
    import numpy as np  # type: ignore

    y_true: List[int] = []
    y_score: List[float] = []
    total_ms = 0.0
    n = 0

    low1 = np.array([0, 120, 180], dtype=np.uint8)
    high1 = np.array([15, 255, 255], dtype=np.uint8)
    low2 = np.array([160, 120, 180], dtype=np.uint8)
    high2 = np.array([179, 255, 255], dtype=np.uint8)

    for s in samples:
        img_path = datasets_dir / s.relpath
        if not img_path.exists():
            continue
        img = cv2.imread(str(img_path))
        if img is None:
            continue

        t0 = time.perf_counter()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        m1 = cv2.inRange(hsv, low1, high1)
        m2 = cv2.inRange(hsv, low2, high2)
        mask = cv2.bitwise_or(m1, m2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, None, iterations=1)
        ratio = float(cv2.countNonZero(mask)) / float(mask.size)
        total_ms += (time.perf_counter() - t0) * 1000
        n += 1

        y_true.append(s.label_fire)
        y_score.append(ratio)

    avg_fps = (1000.0 / (total_ms / n)) if n else 0.0
    log.info("[eval] HSV fire: %d frames, %.1f FPS", n, avg_fps)
    return y_true, y_score, avg_fps


# ---------------------------------------------------------------------------
# Plotting
# ---------------------------------------------------------------------------

def plot_curves(
    y_true: List[int],
    y_score: List[float],
    label: str,
    out_dir: Path,
    suffix: str,
) -> Dict[str, float]:
    """Save ROC, PR, and (at a picked operating point) confusion-matrix PNGs.

    Returns {auc_roc, ap, best_threshold, best_f1}.
    """
    import numpy as np  # type: ignore
    import matplotlib.pyplot as plt  # type: ignore
    from sklearn.metrics import (  # type: ignore
        roc_curve,
        precision_recall_curve,
        auc,
        average_precision_score,
        confusion_matrix,
    )

    yt = np.asarray(y_true)
    ys = np.asarray(y_score, dtype=float)

    # ROC
    fpr, tpr, _ = roc_curve(yt, ys)
    roc_auc = float(auc(fpr, tpr)) if len(fpr) > 1 else 0.0

    fig, ax = plt.subplots(figsize=(5, 5))
    ax.plot(fpr, tpr, label=f"{label} (AUC={roc_auc:.3f})")
    ax.plot([0, 1], [0, 1], "--", color="gray", alpha=0.5)
    ax.set_xlabel("False Positive Rate")
    ax.set_ylabel("True Positive Rate")
    ax.set_title(f"ROC — {label}")
    ax.legend(loc="lower right")
    fig.tight_layout()
    fig.savefig(out_dir / f"roc_{suffix}.png", dpi=150)
    plt.close(fig)

    # PR
    prec, rec, thr = precision_recall_curve(yt, ys)
    ap = float(average_precision_score(yt, ys))

    fig, ax = plt.subplots(figsize=(5, 5))
    ax.plot(rec, prec, label=f"{label} (AP={ap:.3f})")
    ax.set_xlabel("Recall")
    ax.set_ylabel("Precision")
    ax.set_title(f"Precision-Recall — {label}")
    ax.legend(loc="lower left")
    fig.tight_layout()
    fig.savefig(out_dir / f"pr_{suffix}.png", dpi=150)
    plt.close(fig)

    # Operating point: argmax F1 over the PR thresholds.
    best_f1 = 0.0
    best_thr = 0.5
    for p, r, t in zip(prec[:-1], rec[:-1], thr):
        if p + r == 0:
            continue
        f1 = 2 * p * r / (p + r)
        if f1 > best_f1:
            best_f1 = float(f1)
            best_thr = float(t)

    y_pred = (ys >= best_thr).astype(int)
    cm = confusion_matrix(yt, y_pred, labels=[0, 1])

    fig, ax = plt.subplots(figsize=(4, 4))
    im = ax.imshow(cm, cmap="Blues")
    ax.set_xticks([0, 1]); ax.set_yticks([0, 1])
    ax.set_xticklabels(["Neg", "Pos"]); ax.set_yticklabels(["Neg", "Pos"])
    ax.set_xlabel("Predicted"); ax.set_ylabel("Actual")
    ax.set_title(f"Confusion @ thr={best_thr:.2f} — {label}")
    for (i, j), v in np.ndenumerate(cm):
        ax.text(j, i, str(int(v)), ha="center", va="center",
                color="white" if v > cm.max() / 2 else "black")
    fig.colorbar(im, ax=ax, fraction=0.046, pad=0.04)
    fig.tight_layout()
    fig.savefig(out_dir / f"confusion_{suffix}.png", dpi=150)
    plt.close(fig)

    return {
        "auc_roc": roc_auc,
        "ap": ap,
        "best_threshold": best_thr,
        "best_f1": best_f1,
    }


# ---------------------------------------------------------------------------
# Entry
# ---------------------------------------------------------------------------

def run(datasets_dir: Path, out_dir: Path, models: List[str], imgsz: int) -> None:
    out_dir.mkdir(parents=True, exist_ok=True)
    samples = load_manifest(datasets_dir)

    comparison_rows: List[Dict[str, object]] = []
    thresholds: Dict[str, Dict[str, float]] = {}

    # --- Person: one curve set per model
    for model in models:
        y_true, y_score, fps = score_person_yolo(samples, datasets_dir, model, imgsz=imgsz)
        if not y_true:
            log.warning("[eval] skipping %s — no valid samples", model)
            continue
        suffix = f"person_{Path(model).stem}"
        metrics = plot_curves(y_true, y_score, f"Person ({model})", out_dir, suffix)
        metrics["fps"] = fps
        metrics["model"] = model
        metrics["class"] = "person"
        comparison_rows.append(metrics)
        thresholds[suffix] = {"threshold": metrics["best_threshold"]}

    # --- Fire: single HSV segmenter
    y_true_f, y_score_f, fps_f = score_fire_hsv(samples, datasets_dir)
    if y_true_f:
        metrics = plot_curves(y_true_f, y_score_f, "Fire (HSV)", out_dir, "fire_hsv")
        metrics["fps"] = fps_f
        metrics["model"] = "hsv-segmenter"
        metrics["class"] = "fire"
        comparison_rows.append(metrics)
        thresholds["fire_hsv"] = {"threshold": metrics["best_threshold"]}

    # --- Aggregate table
    comp_path = out_dir / "model_comparison.csv"
    with comp_path.open("w", newline="") as f:
        w = csv.DictWriter(
            f,
            fieldnames=["class", "model", "auc_roc", "ap", "best_threshold", "best_f1", "fps"],
        )
        w.writeheader()
        for row in comparison_rows:
            w.writerow({k: row.get(k, "") for k in w.fieldnames})
    log.info("[eval] wrote %s", comp_path)

    thr_path = out_dir / "thresholds.json"
    thr_path.write_text(json.dumps(thresholds, indent=2))
    log.info("[eval] wrote %s", thr_path)


def main(argv: Optional[List[str]] = None) -> int:
    ap = argparse.ArgumentParser(description="Evaluate K-Patrol detector(s)")
    ap.add_argument("--datasets", type=Path, default=Path("detection/datasets"))
    ap.add_argument("--out", type=Path, default=Path("reports"))
    ap.add_argument("--models", nargs="+", default=["yolov8n.pt", "yolov8s.pt"])
    ap.add_argument("--imgsz", type=int, default=320)
    ap.add_argument("-v", "--verbose", action="store_true")
    args = ap.parse_args(argv)

    logging.basicConfig(
        level=logging.DEBUG if args.verbose else logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )
    run(args.datasets, args.out, args.models, args.imgsz)
    return 0


if __name__ == "__main__":
    sys.exit(main())
