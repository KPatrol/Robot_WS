"""TwinLiteNet+ Nano lane-segmentation provider for K-Patrol.

This package wraps an ONNX export of TwinLiteNet+ Nano so the existing
line-follower can swap its HSV thresholding step for a learned binary
lane mask without changing the downstream perspective-transform + PD
pipeline. Designed for Pi 4 CPU inference (~15-25 FPS @ 192x320 INT8 /
FP32 via ONNX Runtime + XNNPACK).

Public API:
    LaneSegProvider — loads ONNX model, exposes infer(bgr_frame) → mask
    LaneSegConfig   — runtime knobs (model path, input size, thresholds)

Repo reference: https://github.com/chequanghuy/TwinLiteNetPlus
Paper: arXiv 2403.16958
"""

from .provider import LaneSegConfig, LaneSegProvider, LaneSegResult  # noqa: F401

__all__ = ["LaneSegConfig", "LaneSegProvider", "LaneSegResult"]
