"""
Temporal smoothing for anomaly detection.

Problem: a single-frame YOLO hit or a momentary HSV flash is noisy and produces
false alerts (red shirt briefly flagged as fire, bounding-box jitter firing the
"person" cooldown on a static chair). For a patrolling robot the cost of a
false alert is low individually but corrosive in aggregate — operators start
ignoring notifications.

Solution: require N-of-M frame consensus before emitting an event.

    smoother = TemporalSmoother(window=5, required=3, class_kinds=("person", "fire"))
    for frame in stream:
        raw_hits = detect(frame)               # {"person": conf, "fire": conf, ...}
        stable   = smoother.update(raw_hits)   # {"person": avg_conf} only once >=3/5
        for kind, conf in stable.items():
            emit(kind, conf)

- Each kind has its own rolling boolean window.
- A kind "fires" only on the transition from below-threshold to at-or-above — so
  downstream cooldown logic (see `AnomalyDetector`) still owns throttling, but
  the smoother prevents flicker from repeatedly tripping the cooldown boundary.
- Confidence reported on fire is the mean of the non-zero confidences in the
  window (so a 3/5 with confidences 0.6/0.8/0.7 reports ~0.7, not the latest).

Lightweight: O(M) memory per kind, no numpy required.
"""

from __future__ import annotations

from collections import deque
from typing import Deque, Dict, Iterable, Mapping, Optional, Tuple, Union


class TemporalSmoother:
    """N-of-M rolling-window consensus per detection kind.

    `required` accepts either a single int (applies to every kind) or a mapping
    of kind→int so different signals can have different debouncing thresholds.
    Per V10 spec: fire requires 3 frames (slower, costlier false-alarm), person
    requires 2 (faster reaction, YOLO already filters most noise).
    """

    def __init__(
        self,
        window: int = 5,
        required: Union[int, Mapping[str, int]] = 3,
        class_kinds: Iterable[str] = ("person", "fire"),
    ):
        if window < 1:
            raise ValueError("window must be >= 1")

        if isinstance(required, Mapping):
            self._default_required = max(required.values()) if required else 1
            self._required_map: Dict[str, int] = dict(required)
        else:
            self._default_required = int(required)
            self._required_map = {}

        if self._default_required < 1:
            raise ValueError("required must be >= 1")
        for k, r in self._required_map.items():
            if r < 1 or r > window:
                raise ValueError(f"required[{k}]={r} must be in [1, window]")
        if self._default_required > window:
            raise ValueError("default required cannot exceed window")

        self.window = window
        # Per-kind rolling window of confidences (0.0 == no hit this frame).
        self._buffers: Dict[str, Deque[float]] = {
            k: deque(maxlen=window) for k in class_kinds
        }
        # Latched state: True once threshold met, reset when below again.
        self._latched: Dict[str, bool] = {k: False for k in self._buffers}

    @property
    def required(self) -> int:
        """Back-compat: returns the default threshold (max of per-kind map)."""
        return self._default_required

    def required_for(self, kind: str) -> int:
        return self._required_map.get(kind, self._default_required)

    def _ensure_kind(self, kind: str) -> None:
        if kind not in self._buffers:
            self._buffers[kind] = deque(maxlen=self.window)
            self._latched[kind] = False

    def update(
        self,
        hits: Dict[str, float],
    ) -> Dict[str, float]:
        """Feed one frame's detections. Returns events that **fire this tick**.

        `hits` maps kind -> confidence (0 or absent means no detection).
        The returned dict only contains kinds that just crossed the
        below -> at-or-above threshold boundary in this update; previously-
        latched kinds that stay above return empty until they drop and re-fire.
        """
        fires: Dict[str, float] = {}

        # Make sure any brand-new kind gets a buffer.
        for kind in hits:
            self._ensure_kind(kind)

        # Push one sample (zero if kind not present) into every tracked buffer.
        for kind, buf in self._buffers.items():
            conf = float(hits.get(kind, 0.0))
            buf.append(conf)

            positive = [c for c in buf if c > 0.0]
            count = len(positive)
            threshold = self.required_for(kind)

            if count >= threshold:
                if not self._latched[kind]:
                    self._latched[kind] = True
                    # Average the non-zero confidences for a stable score.
                    fires[kind] = sum(positive) / count
            else:
                # Drop below threshold -> allow next rise to re-fire.
                self._latched[kind] = False

        return fires

    def reset(self, kind: Optional[str] = None) -> None:
        """Clear buffer(s). Useful after cooldown or mode change."""
        if kind is None:
            for buf in self._buffers.values():
                buf.clear()
            for k in self._latched:
                self._latched[k] = False
        elif kind in self._buffers:
            self._buffers[kind].clear()
            self._latched[kind] = False

    def state(self) -> Dict[str, Tuple[int, int, bool]]:
        """Introspection for tests/logs: {kind: (positive_count, window_size, latched)}."""
        out: Dict[str, Tuple[int, int, bool]] = {}
        for kind, buf in self._buffers.items():
            positive = sum(1 for c in buf if c > 0.0)
            out[kind] = (positive, len(buf), self._latched[kind])
        return out
