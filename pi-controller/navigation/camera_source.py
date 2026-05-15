"""
camera_source.py — Camera abstraction for Raspberry Pi line follow / detection.

Why not just `cv2.VideoCapture(0)`?
    On Raspberry Pi OS Bullseye+ the legacy V4L2 / `bcm2835-v4l2` shim is
    deprecated. The supported stack is `libcamera` via the `picamera2`
    Python bindings. `cv2.VideoCapture` will frequently fail to open or
    return all-zero frames on a Pi 4 / 5 with the official Pi Camera
    Module 2/3 — exactly the failure mode that silently disabled
    LINE_FOLLOW in production.

This module provides:
    * `CameraSource`           — open / read / release abstraction.
    * `Picamera2Source`        — preferred backend on Raspberry Pi.
    * `OpenCVSource`           — fallback for USB UVC webcams or dev box.
    * `ThreadedFrameGrabber`   — daemon thread continuously grabs frames
                                 so the control loop always sees the
                                 newest frame (no head-of-line blocking
                                 from the kernel-side V4L2 buffer).
    * `open_camera(index)`     — factory: tries picamera2 first, then cv2.

Threading rationale: PyImageSearch demonstrated ~2× FPS on Pi-class
hardware by decoupling capture from processing. We adopt the same
pattern but with an additional `latest-frame` semantic so a stalled
processor cannot back-pressure capture or replay stale frames.

Usage:
    cam = open_camera(0, width=640, height=480)
    cam.start()
    while running:
        frame = cam.read()        # always the newest BGR frame, or None
        if frame is None:
            time.sleep(0.01); continue
        ...
    cam.stop()
"""

from __future__ import annotations

import logging
import os
import threading
import time
from abc import ABC, abstractmethod
from typing import Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


# ── Backend availability probes ───────────────────────────────────────────

try:
    from picamera2 import Picamera2  # type: ignore
    _PICAMERA2_AVAILABLE = True
except Exception:
    Picamera2 = None  # type: ignore
    _PICAMERA2_AVAILABLE = False

try:
    import cv2  # type: ignore
    _CV2_AVAILABLE = True
except Exception:
    cv2 = None  # type: ignore
    _CV2_AVAILABLE = False


# ── Abstract base ─────────────────────────────────────────────────────────

class CameraSource(ABC):
    """Common interface: start / read latest BGR frame / stop."""

    backend: str = "abstract"

    @abstractmethod
    def start(self) -> bool: ...
    @abstractmethod
    def read(self) -> Optional[np.ndarray]: ...
    @abstractmethod
    def stop(self) -> None: ...

    def __enter__(self):
        self.start()
        return self

    def __exit__(self, *exc):
        self.stop()


# ── picamera2 backend (preferred on Pi) ───────────────────────────────────

class Picamera2Source(CameraSource):
    backend = "picamera2"

    def __init__(self, width: int = 640, height: int = 480, framerate: int = 30):
        if not _PICAMERA2_AVAILABLE:
            raise RuntimeError("picamera2 not installed")
        self._w = int(width)
        self._h = int(height)
        self._fr = int(framerate)
        self._cam: Optional["Picamera2"] = None  # type: ignore[name-defined]
        self._started = False

    def start(self) -> bool:
        try:
            self._cam = Picamera2()
            # Configure for low-latency video. RGB888 maps directly to a
            # numpy uint8 array; we convert to BGR once per frame to keep
            # the rest of the pipeline OpenCV-native.
            cfg = self._cam.create_video_configuration(
                main={"size": (self._w, self._h), "format": "RGB888"},
                controls={"FrameDurationLimits": (
                    int(1_000_000 / self._fr),  # min  µs
                    int(1_000_000 / self._fr),  # max  µs (lock to target FPS)
                )},
            )
            self._cam.configure(cfg)
            self._cam.start()
            self._started = True
            logger.info(f"[CAM] picamera2 started ({self._w}x{self._h} @ {self._fr} fps)")
            return True
        except Exception as exc:
            logger.error(f"[CAM] picamera2 start failed: {exc}")
            self._cam = None
            self._started = False
            return False

    def read(self) -> Optional[np.ndarray]:
        if not self._started or self._cam is None:
            return None
        try:
            rgb = self._cam.capture_array()
            if rgb is None:
                return None
            # picamera2 RGB888 → OpenCV BGR
            return rgb[:, :, ::-1].copy() if _CV2_AVAILABLE is False \
                else cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        except Exception as exc:
            logger.warning(f"[CAM] picamera2 read failed: {exc}")
            return None

    def stop(self) -> None:
        if self._cam is not None:
            try:
                self._cam.stop()
            except Exception:
                pass
            try:
                self._cam.close()
            except Exception:
                pass
        self._cam = None
        self._started = False


# ── cv2.VideoCapture fallback (USB UVC / dev workstation) ─────────────────

class OpenCVSource(CameraSource):
    backend = "opencv"

    def __init__(self, index: int = 0, width: int = 640, height: int = 480, framerate: int = 30):
        if not _CV2_AVAILABLE:
            raise RuntimeError("cv2 not installed")
        self._idx = int(index)
        self._w = int(width)
        self._h = int(height)
        self._fr = int(framerate)
        self._cap = None
        self._started = False

    def start(self) -> bool:
        cap = cv2.VideoCapture(self._idx)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self._w)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self._h)
        cap.set(cv2.CAP_PROP_FPS, self._fr)
        # Keep buffer at 1 frame so read() always returns the freshest one;
        # not all backends honour this but it's a no-op when unsupported.
        try:
            cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        except Exception:
            pass

        if not cap.isOpened():
            logger.error(f"[CAM] cv2.VideoCapture({self._idx}) failed to open")
            self._cap = None
            return False

        # Warm-up: discard the first frame which on some drivers is black.
        for _ in range(2):
            cap.read()

        self._cap = cap
        self._started = True
        logger.info(f"[CAM] cv2.VideoCapture({self._idx}) opened ({self._w}x{self._h})")
        return True

    def read(self) -> Optional[np.ndarray]:
        if not self._started or self._cap is None:
            return None
        ret, frame = self._cap.read()
        if not ret or frame is None:
            return None
        return frame

    def stop(self) -> None:
        if self._cap is not None:
            try:
                self._cap.release()
            except Exception:
                pass
        self._cap = None
        self._started = False


# ── HTTP MJPEG backend (shares camera with kpatrol-mjpeg.service) ─────────

class HTTPMJPEGSource(CameraSource):
    """Consume an MJPEG-over-HTTP stream produced by another local process.

    On the deployed Pi, kpatrol-mjpeg.service owns /dev/video0 exclusively to
    serve the web UI stream. Picamera2 and cv2 then can't open the camera
    in the control process, which silently disables line-follow + detection.
    Reading frames from the local MJPEG endpoint sidesteps the device lock
    entirely — both services coexist with one capture path.
    """

    backend = "http_mjpeg"

    def __init__(self, url: str, width: int = 640, height: int = 480):
        if not _CV2_AVAILABLE:
            raise RuntimeError("cv2 not available — cannot decode JPEG")
        self._url = url
        self._w = int(width)
        self._h = int(height)
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._frame_seq: int = 0
        self._last_read_seq: int = 0
        self._stream = None

    def _open_stream(self):
        import urllib.request
        return urllib.request.urlopen(self._url, timeout=3.0)

    def start(self) -> bool:
        try:
            self._stream = self._open_stream()
        except Exception as exc:
            logger.error(f"[CAM] HTTP MJPEG open {self._url} failed: {exc}")
            return False
        self._stop_evt.clear()
        self._thread = threading.Thread(
            target=self._reader, daemon=True, name="http_mjpeg",
        )
        self._thread.start()
        logger.info(f"[CAM] HTTP MJPEG source opened: {self._url}")
        return True

    def _reader(self) -> None:
        # Parse multipart MJPEG by scanning for JPEG SOI/EOI markers in the
        # byte stream. Works regardless of the server's boundary string.
        buf = b""
        while not self._stop_evt.is_set():
            if self._stream is None:
                try:
                    self._stream = self._open_stream()
                except Exception as exc:
                    logger.warning(f"[CAM] HTTP MJPEG reopen failed: {exc}")
                    time.sleep(1.0)
                    continue
            try:
                chunk = self._stream.read(8192)
            except Exception as exc:
                logger.warning(f"[CAM] HTTP MJPEG read error: {exc}")
                try:
                    self._stream.close()
                except Exception:
                    pass
                self._stream = None
                continue
            if not chunk:
                time.sleep(0.01)
                continue
            buf += chunk
            # Drop bytes before the first SOI so buf doesn't grow unbounded
            # when the boundary text is large.
            soi = buf.find(b"\xff\xd8")
            if soi == -1:
                if len(buf) > 65536:
                    buf = buf[-1024:]
                continue
            if soi > 0:
                buf = buf[soi:]
            eoi = buf.find(b"\xff\xd9", 2)
            if eoi == -1:
                continue
            jpg = buf[:eoi + 2]
            buf = buf[eoi + 2:]
            arr = np.frombuffer(jpg, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is None:
                continue
            if frame.shape[1] != self._w or frame.shape[0] != self._h:
                frame = cv2.resize(frame, (self._w, self._h))
            with self._lock:
                self._frame = frame
                self._frame_seq += 1

    def read(self) -> Optional[np.ndarray]:
        with self._lock:
            if self._frame is None:
                return None
            # Latest-frame semantic: return each frame at most once so a
            # control loop running faster than the stream doesn't double-
            # count the same frame as a fresh PD step.
            if self._frame_seq == self._last_read_seq:
                return None
            self._last_read_seq = self._frame_seq
            return self._frame

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
            self._thread = None
        if self._stream is not None:
            try:
                self._stream.close()
            except Exception:
                pass
            self._stream = None


# ── Threaded latest-frame wrapper ─────────────────────────────────────────

class ThreadedFrameGrabber(CameraSource):
    """Run a CameraSource in a background thread; `read()` returns the
    newest frame without blocking the caller. Improves end-to-end FPS on
    the Pi by overlapping V4L2/libcamera blocking reads with vision work.
    """

    backend = "threaded"

    def __init__(self, source: CameraSource):
        self._src = source
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._frame_id: int = 0
        self._last_read_id: int = 0
        self._stop_evt = threading.Event()
        self._thread: Optional[threading.Thread] = None

    def start(self) -> bool:
        if not self._src.start():
            return False
        self._stop_evt.clear()
        self._thread = threading.Thread(
            target=self._loop,
            name=f"frame_grabber_{self._src.backend}",
            daemon=True,
        )
        self._thread.start()
        return True

    def _loop(self) -> None:
        while not self._stop_evt.is_set():
            frame = self._src.read()
            if frame is None:
                # Backoff on transient read failure so a disconnected
                # camera doesn't spin a CPU at 100%.
                time.sleep(0.01)
                continue
            with self._lock:
                self._frame = frame
                self._frame_id += 1

    def read(self) -> Optional[np.ndarray]:
        with self._lock:
            if self._frame is None:
                return None
            # Only return frames the consumer hasn't already seen, so a
            # PD loop running faster than the camera doesn't double-count
            # the same frame as two D-term steps.
            if self._frame_id == self._last_read_id:
                return None
            self._last_read_id = self._frame_id
            return self._frame

    def read_blocking(self, timeout: float = 1.0) -> Optional[np.ndarray]:
        """Read but wait up to `timeout` seconds for a fresh frame."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            f = self.read()
            if f is not None:
                return f
            time.sleep(0.005)
        return None

    def stop(self) -> None:
        self._stop_evt.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._src.stop()
        self._thread = None


# ── Factory ───────────────────────────────────────────────────────────────

def open_camera(
    index: int = 0,
    width: int = 640,
    height: int = 480,
    framerate: int = 30,
    prefer: Optional[str] = None,
    threaded: bool = True,
) -> Optional[CameraSource]:
    """Open a camera, preferring picamera2 on Pi and falling back to cv2.

    Args:
        index: cv2 device index (ignored by picamera2 single-camera setups).
        width, height, framerate: capture parameters.
        prefer: "picamera2" / "opencv" / None (auto). Forces a backend.
        threaded: wrap in ThreadedFrameGrabber for latest-frame semantics.

    Returns started CameraSource on success, or None if every backend
    failed. Caller still owns lifecycle: must call .stop() when done.
    """
    # KPATROL_CAMERA_URL lets the deployed Pi share the camera with the
    # local MJPEG service rather than fighting it for /dev/video0.
    http_url = os.environ.get("KPATROL_CAMERA_URL", "").strip()

    candidates: Tuple[str, ...]
    if prefer == "picamera2":
        candidates = ("picamera2",)
    elif prefer == "opencv":
        candidates = ("opencv",)
    elif prefer == "http_mjpeg":
        candidates = ("http_mjpeg",)
    elif http_url:
        candidates = ("http_mjpeg", "picamera2", "opencv")
    else:
        candidates = ("picamera2", "opencv")

    for backend in candidates:
        try:
            if backend == "picamera2" and _PICAMERA2_AVAILABLE:
                src: CameraSource = Picamera2Source(width, height, framerate)
            elif backend == "opencv" and _CV2_AVAILABLE:
                src = OpenCVSource(index, width, height, framerate)
            elif backend == "http_mjpeg" and _CV2_AVAILABLE and http_url:
                src = HTTPMJPEGSource(http_url, width, height)
            else:
                continue
        except Exception as exc:
            logger.warning(f"[CAM] {backend} construct failed: {exc}")
            continue

        # HTTPMJPEGSource manages its own reader thread and latest-frame
        # dedup, so wrapping it in ThreadedFrameGrabber would spin an extra
        # thread for no benefit. Leave it unwrapped.
        if backend == "http_mjpeg":
            wrapper: CameraSource = src
        else:
            wrapper = ThreadedFrameGrabber(src) if threaded else src
        if wrapper.start():
            return wrapper

    logger.error("[CAM] no camera backend opened — line follow / detection disabled")
    return None


__all__ = [
    "CameraSource",
    "Picamera2Source",
    "OpenCVSource",
    "ThreadedFrameGrabber",
    "open_camera",
]
