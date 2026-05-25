"""ONNX-Runtime inference wrapper for TwinLiteNet+ lane segmentation.

TwinLiteNet+ is a tiny two-headed segmentation network from the paper at
arXiv 2403.16958. We use only the lane-line head; the drivable-area head
output is ignored because the K-Patrol indoor patrol robot follows
tape/marker lines on a corridor floor — the open-floor mask carries no
useful steering signal for that task.

Pi 4 CPU benchmarks (target, to be confirmed by benchmark_lane_seg.py):
    Nano+ ONNX FP32  @ 192x320 → ~15-20 FPS
    Nano+ ONNX FP16  @ 192x320 → ~20-25 FPS  (if ONNX Runtime supports it)
    Small+ ONNX INT8 @ 256x384 → ~8-12 FPS

The provider stays I/O-free at import time so the parent line-follower
can construct it lazily and fall back to HSV if the ONNX model file is
missing — keeps the deployment chain "soft": flashing a Pi without the
ONNX file still ships a working robot, just with rule-based vision.
"""

from __future__ import annotations

import logging
import os
import time
from dataclasses import dataclass, field
from typing import Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)

# onnxruntime is imported lazily so the rest of the pi-controller doesn't
# pull a ~50 MB shared object into memory just to render the line-follower
# config. The provider raises a clear error at construct time if the user
# asks for ONNX inference without the runtime installed.
try:
    import onnxruntime as ort  # type: ignore
    _ORT_AVAILABLE = True
except ImportError:  # pragma: no cover - depends on Pi venv
    _ORT_AVAILABLE = False
    ort = None  # type: ignore


@dataclass
class LaneSegConfig:
    """Runtime tuning knobs for the lane-seg provider.

    Defaults target Nano+ @ 192x320 — the smallest variant that still
    produces a usable lane mask on indoor corridor video. Larger inputs
    are configurable but mind the FPS cliff on Pi 4 CPU."""

    # Absolute path to the exported ONNX file. Resolved at construct time
    # so the consumer can keep the file out of the git repo and ship it
    # via rsync to the Pi.
    model_path: str = "/home/khoavd/kpatrol/pi-controller/models/twinlitenet_nano_192x320.onnx"

    # Network input size — match the export. (height, width) in pixels.
    # Common picks: (192, 320), (256, 384), (384, 640).
    input_size: Tuple[int, int] = (192, 320)

    # Sigmoid threshold for the lane head. The TwinLiteNet+ paper uses
    # argmax over 2 channels but a single sigmoid gives a smoother mask
    # for the downstream centroid step. 0.5 is the natural midpoint;
    # raise it (e.g. 0.6) if the model hallucinates lane pixels on dirty
    # floors, lower (0.35) for faint tape lines.
    lane_threshold: float = 0.5

    # ONNX Runtime providers in priority order. XNNPACK lights up the Pi 4
    # NEON path automatically when present; otherwise the CPU provider
    # falls back to the reference kernels. Order is honoured by ORT.
    ort_providers: Tuple[str, ...] = field(
        default_factory=lambda: ("XnnpackExecutionProvider", "CPUExecutionProvider")
    )

    # Intra-op thread count. Pi 4 has 4 cores; reserving 2 for inference
    # leaves enough for the camera grab thread + Pi-controller main loop
    # without GIL thrash.
    intra_op_threads: int = 2

    # Optional ROI crop applied BEFORE resize. Useful when the camera FOV
    # includes ceiling / dashboard / robot chassis edges that aren't part
    # of the floor. Tuple is (y0, y1, x0, x1) in source-frame fractions
    # 0..1. None = full frame.
    roi_frac: Optional[Tuple[float, float, float, float]] = None

    # Use FP16 weights at runtime when available. Slightly faster on Pi
    # if the ONNX file was exported with FP16 — leave False for FP32.
    fp16: bool = False


@dataclass
class LaneSegResult:
    """One pass of inference output, packaged for the line-follower."""

    # Binary lane mask in *source-frame* coordinates (uint8 0/255).
    # Already upsampled and ROI-uncropped, so the consumer can BEV-warp
    # it with the existing perspective transform exactly as it would the
    # HSV mask. Same shape as the input frame's first two axes.
    mask: np.ndarray

    # Raw probability map (float32 0..1) at network resolution. Exposed
    # mainly for telemetry / debug overlay — the binary mask above is the
    # one to feed into the PD pipeline.
    prob: np.ndarray

    # Wall-clock latency in milliseconds for this single inference call,
    # measured from the start of preprocessing through the end of the
    # postprocess upsample. Useful for the watchdog: when latency
    # consistently exceeds 1 / desired_fps the line-follower can throttle
    # the camera tick down to avoid GIL starvation elsewhere.
    latency_ms: float


class LaneSegProvider:
    """Loads a TwinLiteNet+ ONNX model and exposes a stateless infer().

    The provider keeps no per-frame state — the downstream line-follower
    owns the PD / temporal smoothing. This isolation lets us hot-swap the
    underlying model (different variant, different export) without
    touching the navigation code.

    A missing or corrupt model file does NOT raise at construct time; it
    sets `self.available = False` so the line-follower can detect and
    fall back to HSV. This matches the existing soft-fail pattern around
    the BNO08x and the GPS reader.
    """

    def __init__(self, cfg: Optional[LaneSegConfig] = None):
        self.cfg = cfg or LaneSegConfig()
        self.available = False
        self._session: Optional["ort.InferenceSession"] = None
        self._input_name: str = ""
        self._output_names: Tuple[str, ...] = ()

        if not _ORT_AVAILABLE:
            logger.warning("[LaneSeg] onnxruntime not installed — provider disabled. "
                          "Install with: pip install onnxruntime")
            return
        if not os.path.exists(self.cfg.model_path):
            logger.warning(f"[LaneSeg] Model not found at {self.cfg.model_path} — "
                          "provider disabled. Run setup_twinlite.sh on the Pi to fetch it.")
            return

        try:
            so = ort.SessionOptions()
            so.intra_op_num_threads = self.cfg.intra_op_threads
            so.graph_optimization_level = ort.GraphOptimizationLevel.ORT_ENABLE_ALL
            # Some Pi-installed wheels don't bundle XNNPACK — filter the
            # provider list to what's actually available to avoid a noisy
            # warning every frame.
            available_providers = set(ort.get_available_providers())
            providers = [p for p in self.cfg.ort_providers if p in available_providers]
            if not providers:
                providers = ["CPUExecutionProvider"]
            self._session = ort.InferenceSession(
                self.cfg.model_path,
                sess_options=so,
                providers=providers,
            )
            self._input_name = self._session.get_inputs()[0].name
            self._output_names = tuple(o.name for o in self._session.get_outputs())
            self.available = True
            logger.info(
                f"[LaneSeg] Loaded {os.path.basename(self.cfg.model_path)} "
                f"@ {self.cfg.input_size} via {providers}"
            )
        except Exception as exc:
            logger.error(f"[LaneSeg] Failed to load ONNX model: {exc}")
            self._session = None

    # ─── Preprocess ────────────────────────────────────────────────────
    def _preprocess(self, bgr: np.ndarray) -> Tuple[np.ndarray, Tuple[int, int, int, int]]:
        """BGR uint8 (H, W, 3) → float32 (1, 3, h, w) in 0..1, BGR→RGB swap.

        Returns the preprocessed tensor plus the (y0, y1, x0, x1) crop the
        postprocess step needs to paste the upsampled mask back into the
        source frame coordinate space.
        """
        h, w = bgr.shape[:2]
        if self.cfg.roi_frac is not None:
            y0f, y1f, x0f, x1f = self.cfg.roi_frac
            y0 = max(0, int(y0f * h))
            y1 = min(h, int(y1f * h))
            x0 = max(0, int(x0f * w))
            x1 = min(w, int(x1f * w))
            crop = bgr[y0:y1, x0:x1]
        else:
            y0, y1, x0, x1 = 0, h, 0, w
            crop = bgr

        # cv2.resize is INTER_LINEAR by default — matches the reference
        # TwinLiteNet preprocessing in the upstream repo.
        import cv2  # type: ignore
        net_h, net_w = self.cfg.input_size
        resized = cv2.resize(crop, (net_w, net_h), interpolation=cv2.INTER_LINEAR)

        # BGR → RGB, HWC → CHW, uint8 → float32 [0, 1]. The reference
        # PyTorch model does not subtract ImageNet mean (verified in the
        # repo's test.py preprocessing).
        rgb = resized[..., ::-1]
        chw = np.transpose(rgb, (2, 0, 1)).astype(
            np.float16 if self.cfg.fp16 else np.float32
        )
        chw = chw * (1.0 / 255.0)
        tensor = np.expand_dims(chw, axis=0)
        return tensor, (y0, y1, x0, x1)

    # ─── Postprocess ───────────────────────────────────────────────────
    def _postprocess(
        self,
        outputs: Tuple[np.ndarray, ...],
        crop: Tuple[int, int, int, int],
        source_hw: Tuple[int, int],
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Network output → (binary mask source-frame, probability map net-res).

        TwinLiteNet+ has two output heads. We assume the lane-line head is
        either the second output (`outputs[1]`) — which matches the
        upstream `model.forward()` return order — or any output whose
        last channel dim is 2 (drivable[ch=0] + lane[ch=1]).
        """
        import cv2  # type: ignore

        # Pick the lane head. The repo's forward returns (drivable, lane).
        if len(outputs) >= 2:
            lane_logits = outputs[1]
        else:
            # Single-output variant — assume channel index 1 is lane.
            lane_logits = outputs[0]

        # Shape: (1, 2, H, W) for argmax-style head or (1, 1, H, W) for
        # sigmoid head. Handle both.
        if lane_logits.ndim != 4:
            raise ValueError(f"[LaneSeg] unexpected lane output shape: {lane_logits.shape}")
        if lane_logits.shape[1] == 2:
            # Two-channel softmax; lane is channel 1.
            # Use a numerically stable softmax restricted to the lane vs.
            # background logit pair.
            e0 = np.exp(lane_logits[0, 0] - np.maximum(lane_logits[0, 0], lane_logits[0, 1]))
            e1 = np.exp(lane_logits[0, 1] - np.maximum(lane_logits[0, 0], lane_logits[0, 1]))
            prob = e1 / (e0 + e1)
        elif lane_logits.shape[1] == 1:
            # Sigmoid head.
            prob = 1.0 / (1.0 + np.exp(-lane_logits[0, 0]))
        else:
            raise ValueError(f"[LaneSeg] unsupported lane channel count: {lane_logits.shape[1]}")

        prob = prob.astype(np.float32)
        bin_small = (prob >= self.cfg.lane_threshold).astype(np.uint8) * 255

        # Upsample to crop size, then paste into a full-source-frame
        # zero-initialised mask so downstream BEV warp gets back exactly
        # the dimensions the camera produced.
        y0, y1, x0, x1 = crop
        crop_h, crop_w = y1 - y0, x1 - x0
        upsampled = cv2.resize(bin_small, (crop_w, crop_h), interpolation=cv2.INTER_NEAREST)
        full = np.zeros(source_hw, dtype=np.uint8)
        full[y0:y1, x0:x1] = upsampled
        return full, prob

    # ─── Public API ────────────────────────────────────────────────────
    def infer(self, bgr_frame: np.ndarray) -> Optional[LaneSegResult]:
        """Run one inference pass. Returns None if the provider isn't ready.

        Threadsafe-by-coincidence: ONNX Runtime is internally thread-safe
        on the same session. Don't share self.cfg across mutating callers.
        """
        if not self.available or self._session is None:
            return None
        t0 = time.perf_counter()
        tensor, crop = self._preprocess(bgr_frame)
        outputs = self._session.run(self._output_names, {self._input_name: tensor})
        mask, prob = self._postprocess(
            tuple(outputs),
            crop,
            (bgr_frame.shape[0], bgr_frame.shape[1]),
        )
        latency_ms = (time.perf_counter() - t0) * 1000.0
        return LaneSegResult(mask=mask, prob=prob, latency_ms=latency_ms)
