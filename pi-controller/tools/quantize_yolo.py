"""
One-shot YOLO ONNX dynamic-quantization to INT8 for Pi 4 inference.

Why this script:
    Ultralytics' `export(int8=True)` needs a calibration dataset and downloads
    coco128.yaml. On a constrained Pi that's slow and brittle. ONNX Runtime's
    `quantize_dynamic` quantizes only weights (activations stay float), needs
    no calibration data, and still delivers ~3× size reduction + ~1.5–2× CPU
    speedup on Pi 4 — the right trade for a one-time edge deploy.

Run:
    # On Pi (or any machine with onnxruntime ≥1.17):
    python -m tools.quantize_yolo \
        --input yolov8n.onnx \
        --output yolov8n_int8.onnx

    # Or pass --auto to discover and quantize the configured model.
    python -m tools.quantize_yolo --auto

After this runs, anomaly_detector._resolve_yolo_model_path() automatically
prefers the *_int8.onnx variant if present. Delete the int8 file to roll back.
"""

from __future__ import annotations

import argparse
import logging
import os
import sys
import time
from pathlib import Path

log = logging.getLogger("kpatrol.quantize_yolo")


def _quantize(input_path: str, output_path: str) -> None:
    try:
        from onnxruntime.quantization import quantize_dynamic, QuantType
    except ImportError as exc:
        raise SystemExit(
            "onnxruntime.quantization not available. "
            "Install with: pip install 'onnxruntime>=1.17.3'"
        ) from exc

    log.info("[quantize] %s -> %s (dynamic INT8)", input_path, output_path)
    start = time.perf_counter()
    quantize_dynamic(
        model_input=input_path,
        model_output=output_path,
        weight_type=QuantType.QInt8,
        per_channel=False,
        reduce_range=False,
    )
    elapsed = time.perf_counter() - start
    src_size = os.path.getsize(input_path) / (1024 * 1024)
    dst_size = os.path.getsize(output_path) / (1024 * 1024)
    log.info(
        "[quantize] done in %.1fs — size %.1f MB -> %.1f MB (%.1f%% reduction)",
        elapsed, src_size, dst_size, (1 - dst_size / src_size) * 100,
    )


def _smoke_check(model_path: str) -> None:
    """Cheap sanity check: load and run one dummy inference."""
    try:
        import numpy as np
        import onnxruntime as ort
    except ImportError:
        log.warning("[quantize] skipping smoke check (onnxruntime/numpy missing)")
        return

    sess = ort.InferenceSession(model_path, providers=["CPUExecutionProvider"])
    inp = sess.get_inputs()[0]
    # YOLOv8 ONNX expects 1x3xHxW float32
    shape = tuple(d if isinstance(d, int) else 1 for d in inp.shape)
    dummy = np.random.rand(*shape).astype(np.float32)
    start = time.perf_counter()
    sess.run(None, {inp.name: dummy})
    log.info("[quantize] smoke inference %.1f ms (shape=%s)",
             (time.perf_counter() - start) * 1000, shape)


def _resolve_auto_input() -> str:
    """Default discovery: <controller>/yolov8n.onnx beside this tools dir."""
    here = Path(__file__).resolve()
    controller_dir = here.parent.parent  # tools/ -> pi-controller/
    onnx = controller_dir / "yolov8n.onnx"
    if onnx.exists():
        return str(onnx)
    pt = controller_dir / "yolov8n.pt"
    if pt.exists():
        raise SystemExit(
            f"Only .pt found at {pt}. Run AnomalyDetector once first to "
            "auto-export the FP32 .onnx, then re-run this quantizer."
        )
    raise SystemExit(
        f"No yolov8n.onnx found at {controller_dir}. Provide --input explicitly."
    )


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--input", help="Path to FP32 .onnx (omit with --auto)")
    ap.add_argument("--output", help="Output path (defaults to <input>_int8.onnx)")
    ap.add_argument("--auto", action="store_true",
                    help="Discover the FP32 model beside pi-controller/")
    ap.add_argument("--no-smoke", action="store_true",
                    help="Skip post-quantization smoke inference")
    args = ap.parse_args(argv)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    if args.auto:
        input_path = _resolve_auto_input()
    elif args.input:
        input_path = args.input
    else:
        ap.error("must provide --input PATH or --auto")
        return 2

    if not os.path.exists(input_path):
        raise SystemExit(f"input not found: {input_path}")

    if args.output:
        output_path = args.output
    else:
        base, ext = os.path.splitext(input_path)
        output_path = f"{base}_int8{ext}"

    _quantize(input_path, output_path)
    if not args.no_smoke:
        try:
            _smoke_check(output_path)
        except Exception as exc:
            log.warning("[quantize] smoke check failed: %s", exc)

    log.info("[quantize] INT8 model ready: %s", output_path)
    log.info(
        "[quantize] AnomalyDetector will auto-pick it up on next start "
        "(yolo_model_prefer_onnx must remain True)."
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
