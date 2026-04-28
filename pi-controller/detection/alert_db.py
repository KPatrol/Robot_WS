"""
Local SQLite persistence for detection alerts.

Runs on the Pi next to the detector. Two jobs:

1. **Offline resilience** — if the MQTT broker or the NestJS backend is down,
   alerts are still logged locally with `synced=0`. When the bridge reconnects
   it drains the backlog in timestamp order.

2. **Thesis evidence** — a durable record of every trigger the system fired,
   with confidence + snapshot path, so the evaluation chapter can tabulate
   real-world detections (not just the eval-set numbers).

The schema is deliberately thin. The authoritative store is the NestJS
`Alert` table backed by Postgres; this is just a write-ahead-log.

Usage:
    store = AlertStore("alerts.db")
    store.insert(kind="person", confidence=0.91, bbox=(..), snapshot="...", ts=...)
    for row in store.unsynced(limit=100):
        if publish_to_mqtt(row):
            store.mark_synced(row["id"])
"""

from __future__ import annotations

import json
import sqlite3
import threading
import time
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple


_SCHEMA = """
CREATE TABLE IF NOT EXISTS alerts (
    id          INTEGER PRIMARY KEY AUTOINCREMENT,
    ts          REAL    NOT NULL,
    kind        TEXT    NOT NULL,
    confidence  REAL    NOT NULL,
    bbox_json   TEXT    NOT NULL,
    frame_w     INTEGER NOT NULL,
    frame_h     INTEGER NOT NULL,
    snapshot    TEXT    NOT NULL,
    robot       TEXT    NOT NULL,
    extra_json  TEXT,
    synced      INTEGER NOT NULL DEFAULT 0,
    synced_at   REAL
);
CREATE INDEX IF NOT EXISTS idx_alerts_synced ON alerts(synced, ts);
CREATE INDEX IF NOT EXISTS idx_alerts_ts      ON alerts(ts);
"""


class AlertStore:
    """Thread-safe SQLite WAL for detection alerts."""

    def __init__(self, path: str = "alerts.db"):
        self.path = path
        Path(path).parent.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock()
        # `check_same_thread=False` is safe given our external lock.
        self._conn = sqlite3.connect(path, check_same_thread=False, isolation_level=None)
        self._conn.row_factory = sqlite3.Row
        # WAL keeps writers from blocking the reader (the sync drainer).
        self._conn.execute("PRAGMA journal_mode=WAL;")
        self._conn.execute("PRAGMA synchronous=NORMAL;")
        self._conn.executescript(_SCHEMA)

    # ------------------------------------------------------------------
    # Write path
    # ------------------------------------------------------------------

    def insert(
        self,
        *,
        kind: str,
        confidence: float,
        bbox: Tuple[int, int, int, int],
        snapshot: str,
        robot: str,
        frame_w: int,
        frame_h: int,
        ts: Optional[float] = None,
        extra: Optional[Dict[str, Any]] = None,
    ) -> int:
        ts = ts if ts is not None else time.time()
        with self._lock:
            cur = self._conn.execute(
                """
                INSERT INTO alerts (ts, kind, confidence, bbox_json, frame_w, frame_h,
                                    snapshot, robot, extra_json)
                VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?)
                """,
                (
                    ts,
                    kind,
                    float(confidence),
                    json.dumps(list(bbox)),
                    int(frame_w),
                    int(frame_h),
                    snapshot,
                    robot,
                    json.dumps(extra) if extra else None,
                ),
            )
            return int(cur.lastrowid)

    # ------------------------------------------------------------------
    # Sync drainer
    # ------------------------------------------------------------------

    def unsynced(self, limit: int = 100) -> List[sqlite3.Row]:
        with self._lock:
            cur = self._conn.execute(
                "SELECT * FROM alerts WHERE synced=0 ORDER BY ts ASC LIMIT ?",
                (int(limit),),
            )
            return list(cur.fetchall())

    def mark_synced(self, alert_id: int) -> None:
        with self._lock:
            self._conn.execute(
                "UPDATE alerts SET synced=1, synced_at=? WHERE id=?",
                (time.time(), int(alert_id)),
            )

    # ------------------------------------------------------------------
    # Housekeeping / stats
    # ------------------------------------------------------------------

    def counts_by_kind(self, since_ts: Optional[float] = None) -> Dict[str, int]:
        q = "SELECT kind, COUNT(*) AS n FROM alerts"
        args: Iterable[Any] = ()
        if since_ts is not None:
            q += " WHERE ts >= ?"
            args = (since_ts,)
        q += " GROUP BY kind"
        with self._lock:
            cur = self._conn.execute(q, tuple(args))
            return {row["kind"]: int(row["n"]) for row in cur.fetchall()}

    def prune_synced_older_than(self, age_sec: float) -> int:
        cutoff = time.time() - age_sec
        with self._lock:
            cur = self._conn.execute(
                "DELETE FROM alerts WHERE synced=1 AND ts < ?",
                (cutoff,),
            )
            return int(cur.rowcount or 0)

    def close(self) -> None:
        with self._lock:
            self._conn.close()
