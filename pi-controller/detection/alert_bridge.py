"""
Alert bridge: glues AnomalyDetector to MQTT.

Publishes to: kpatrol/{serial}/alert
Payload:
    {
      "kind": "person|fire|motion",
      "confidence": 0.92,
      "bbox": [x, y, w, h],
      "ts": 1719000000.0,
      "snapshot": "snapshots/1719000000_person.jpg",
      "robot": "KPATROL-001"
    }

Run:
    python -m detection.alert_bridge [--dry-run]
"""

from __future__ import annotations

import argparse
import json
import logging
import os
import signal
import sys
import threading
import time
from dataclasses import asdict

try:
    import paho.mqtt.client as mqtt
except ImportError:
    mqtt = None  # type: ignore

from typing import Optional

from .alert_db import AlertStore
from .anomaly_detector import AnomalyDetector, DetectionConfig, DetectionEvent

log = logging.getLogger("kpatrol.alert_bridge")


def load_mqtt_env(path: str = "mqtt.env") -> dict:
    """Minimal .env loader (KEY=VALUE per line)."""
    out: dict = {}
    if not os.path.exists(path):
        return out
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#") or "=" not in line:
                continue
            k, v = line.split("=", 1)
            out[k.strip()] = v.strip()
    return out


class AlertBridge:
    def __init__(self, env: dict, dry_run: bool = False, db_path: str = "alerts.db"):
        self.env = env
        self.dry_run = dry_run
        self.serial = env.get("ROBOT_SERIAL", "KPATROL-001")
        self.topic = f"{env.get('MQTT_TOPIC_PREFIX', 'kpatrol')}/{self.serial}/alert"
        self.client: "mqtt.Client | None" = None
        self.store = AlertStore(db_path)
        self._drain_stop = threading.Event()
        self._drain_thread: Optional[threading.Thread] = None
        # Reconnect state: when the initial connect fails we keep a "shell"
        # client around and let the drainer thread retry on each tick. paho's
        # loop_start handles reconnects only after a successful first connect,
        # so we own the cold-start retry path ourselves.
        self._connect_pending: bool = False
        self._last_connect_attempt: float = 0.0
        self._connect_retry_sec: float = 10.0

    def _build_client(self) -> "mqtt.Client":
        c = mqtt.Client(client_id=f"{self.serial}-alert")
        user = self.env.get("MQTT_USERNAME")
        pwd = self.env.get("MQTT_PASSWORD")
        if user:
            c.username_pw_set(user, pwd)
        c.on_disconnect = self._on_disconnect
        c.on_connect = self._on_connect
        return c

    def _on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            log.info("[bridge] mqtt connected")
            self._connect_pending = False
        else:
            log.warning("[bridge] mqtt connect rc=%s", rc)

    def _on_disconnect(self, client, userdata, rc):
        # rc != 0 means unexpected drop; paho will auto-reconnect via loop_start.
        if rc != 0:
            log.warning("[bridge] mqtt disconnected rc=%s — will auto-reconnect", rc)

    def connect(self) -> None:
        if mqtt is None:
            log.warning("paho-mqtt not installed — events will be printed only")
            return
        c = self._build_client()
        host = self.env.get("MQTT_HOST", "localhost")
        port = int(self.env.get("MQTT_PORT", "1883"))
        log.info("[bridge] connecting to %s:%d", host, port)
        self._last_connect_attempt = time.time()
        try:
            c.connect(host, port, keepalive=30)
        except OSError as exc:
            # Broker down at startup — keep events flowing into SQLite and let
            # the drainer retry. Without this, a delayed broker bring-up would
            # crash the whole detection process.
            log.warning("[bridge] initial connect failed: %s — buffering to disk", exc)
            self._connect_pending = True
        c.loop_start()
        self.client = c

    def _try_reconnect(self) -> None:
        """Retry an initial connect that failed at startup.

        Called from the drainer tick. paho's auto-reconnect only kicks in once
        a session has been established at least once; cold-start failures stay
        cold until we explicitly reconnect.
        """
        if not self._connect_pending or self.client is None:
            return
        now = time.time()
        if now - self._last_connect_attempt < self._connect_retry_sec:
            return
        self._last_connect_attempt = now
        host = self.env.get("MQTT_HOST", "localhost")
        port = int(self.env.get("MQTT_PORT", "1883"))
        try:
            self.client.reconnect()
            log.info("[bridge] reconnect attempt to %s:%d issued", host, port)
        except OSError as exc:
            log.debug("[bridge] reconnect still failing: %s", exc)

    def disconnect(self) -> None:
        self._drain_stop.set()
        if self._drain_thread is not None:
            self._drain_thread.join(timeout=2.0)
            self._drain_thread = None
        if self.client is not None:
            self.client.loop_stop()
            self.client.disconnect()
            self.client = None
        self.store.close()

    def publish(self, event: DetectionEvent) -> None:
        # Always persist first — durable record regardless of MQTT state.
        # snapshot_b64 lives in extra_json so the drainer can replay it later
        # without re-reading the JPEG from disk (which may be rotated by then).
        extra = {"snapshot_b64": event.snapshot_b64} if event.snapshot_b64 else None
        alert_id = self.store.insert(
            kind=event.kind,
            confidence=event.confidence,
            bbox=tuple(event.bbox),
            snapshot=event.snapshot_path,
            robot=self.serial,
            frame_w=event.frame_width,
            frame_h=event.frame_height,
            ts=event.timestamp,
            extra=extra,
        )
        if self._publish_row(alert_id, event.kind, event.confidence, tuple(event.bbox),
                             event.timestamp, event.snapshot_path,
                             event.frame_width, event.frame_height,
                             snapshot_b64=event.snapshot_b64):
            self.store.mark_synced(alert_id)

    def _publish_row(
        self,
        alert_id: int,
        kind: str,
        confidence: float,
        bbox: tuple,
        ts: float,
        snapshot: str,
        frame_w: int,
        frame_h: int,
        snapshot_b64: str = "",
    ) -> bool:
        payload = {
            "id": alert_id,
            "kind": kind,
            "confidence": round(confidence, 3),
            "bbox": list(bbox),
            "ts": ts,
            "snapshot": snapshot,
            "robot": self.serial,
            "frame_size": [frame_w, frame_h],
        }
        if snapshot_b64:
            payload["snapshot_b64"] = snapshot_b64
        body = json.dumps(payload, separators=(",", ":"))
        if self.client is None or not self.client.is_connected():
            log.info("[bridge] offline, queued id=%d kind=%s", alert_id, kind)
            return False
        try:
            info = self.client.publish(self.topic, body, qos=1, retain=False)
        except (OSError, ValueError) as exc:
            # Socket can drop mid-publish; surface and let the drainer retry.
            log.warning("[bridge] publish raised: %s — leaving id=%d unsynced", exc, alert_id)
            return False
        log.info("[bridge] %s -> %s", self.topic, body)
        # paho rc==0 means accepted onto the outbound buffer; QoS=1 will retry.
        return getattr(info, "rc", 0) == 0

    # ------------------------------------------------------------------
    # Backlog drainer: retries unsynced alerts when the broker returns.
    # ------------------------------------------------------------------

    def start_drainer(self, interval_sec: float = 5.0) -> None:
        if self._drain_thread is not None:
            return
        self._drain_stop.clear()
        self._drain_thread = threading.Thread(
            target=self._drain_loop, args=(interval_sec,), daemon=True,
        )
        self._drain_thread.start()

    def _drain_loop(self, interval_sec: float) -> None:
        # Run maintenance on a longer cadence than the drain tick — pruning the
        # synced backlog and capping the unsynced one is cheap but pointless to
        # do every 5 s. VACUUM is reclaim-only and slower, so it runs daily.
        maintenance_period_sec = 600.0
        vacuum_period_sec = 24 * 3600.0
        last_maintenance = 0.0
        last_vacuum = 0.0
        while not self._drain_stop.wait(interval_sec):
            now = time.time()
            if now - last_maintenance >= maintenance_period_sec:
                try:
                    do_vacuum = (now - last_vacuum) >= vacuum_period_sec
                    stats = self.store.maintenance(vacuum=do_vacuum)
                    if stats["pruned"] or stats["capped"] or do_vacuum:
                        log.info(
                            "[bridge] maintenance pruned=%d capped=%d vacuum=%s",
                            stats["pruned"], stats["capped"], do_vacuum,
                        )
                    last_maintenance = now
                    if do_vacuum:
                        last_vacuum = now
                except Exception as exc:
                    log.warning("[bridge] maintenance failed: %s", exc)
                    last_maintenance = now  # don't hammer a broken store

            # Cold-start retry: if the initial connect failed, keep poking the
            # broker. We do this before the connectedness check so a successful
            # reconnect is acted on in the same tick.
            if self._connect_pending:
                self._try_reconnect()
            if self.client is None or not self.client.is_connected():
                continue
            try:
                rows = self.store.unsynced(limit=50)
            except Exception as exc:
                log.warning("[bridge] drain read failed: %s", exc)
                continue
            for row in rows:
                try:
                    bbox = tuple(json.loads(row["bbox_json"]))
                except Exception:
                    bbox = (0, 0, 0, 0)
                snapshot_b64 = ""
                extra_raw = row["extra_json"] if "extra_json" in row.keys() else None
                if extra_raw:
                    try:
                        snapshot_b64 = json.loads(extra_raw).get("snapshot_b64", "") or ""
                    except Exception:
                        snapshot_b64 = ""
                try:
                    ok = self._publish_row(
                        row["id"], row["kind"], row["confidence"], bbox,
                        row["ts"], row["snapshot"], row["frame_w"], row["frame_h"],
                        snapshot_b64=snapshot_b64,
                    )
                except Exception as exc:
                    # Belt-and-braces: never let one bad row kill the drainer.
                    log.warning("[bridge] drain publish raised on id=%s: %s", row.get("id"), exc)
                    break
                if ok:
                    self.store.mark_synced(row["id"])
                else:
                    break  # broker dropped again — try next tick


def main(argv=None) -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--env", default="mqtt.env")
    ap.add_argument("--camera", type=int, default=0)
    ap.add_argument("--snapshots", default="snapshots")
    ap.add_argument("--db", default="alerts.db", help="SQLite write-ahead log path")
    args = ap.parse_args(argv)

    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(name)s %(levelname)s %(message)s",
    )

    env = load_mqtt_env(args.env)
    bridge = AlertBridge(env, dry_run=args.dry_run, db_path=args.db)
    bridge.connect()
    bridge.start_drainer()

    cfg = DetectionConfig(
        dry_run=args.dry_run,
        camera_index=args.camera,
        snapshot_dir=args.snapshots,
        robot_serial=env.get("ROBOT_SERIAL", "KPATROL-001"),
    )
    detector = AnomalyDetector(cfg, on_event=bridge.publish)

    def _shutdown(signum, frame):
        log.info("[bridge] signal %s, shutting down", signum)
        detector.stop()
        bridge.disconnect()
        sys.exit(0)

    signal.signal(signal.SIGINT, _shutdown)
    signal.signal(signal.SIGTERM, _shutdown)

    detector.start(blocking=True)
    bridge.disconnect()
    return 0


if __name__ == "__main__":
    sys.exit(main())
