"""Unit tests for AlertStore (SQLite write-ahead log)."""

from __future__ import annotations

import os
import tempfile
import unittest

from .alert_db import AlertStore


class AlertStoreCore(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.NamedTemporaryFile(suffix=".db", delete=False)
        self.tmp.close()
        self.store = AlertStore(self.tmp.name)

    def tearDown(self):
        self.store.close()
        os.unlink(self.tmp.name)

    def _insert(self, kind="person", conf=0.9, ts=None):
        return self.store.insert(
            kind=kind, confidence=conf, bbox=(10, 20, 30, 40),
            snapshot=f"/tmp/{kind}.jpg", robot="KPATROL-001",
            frame_w=640, frame_h=480, ts=ts,
        )

    def test_insert_and_unsynced(self):
        a = self._insert("person", 0.9)
        b = self._insert("fire", 0.05)
        rows = self.store.unsynced()
        self.assertEqual(len(rows), 2)
        ids = [r["id"] for r in rows]
        self.assertEqual(sorted(ids), sorted([a, b]))

    def test_mark_synced_excludes_from_backlog(self):
        a = self._insert("person", 0.9)
        self.store.mark_synced(a)
        self.assertEqual(len(self.store.unsynced()), 0)

    def test_counts_by_kind(self):
        self._insert("person")
        self._insert("person")
        self._insert("fire")
        c = self.store.counts_by_kind()
        self.assertEqual(c, {"person": 2, "fire": 1})

    def test_prune_old_synced(self):
        a = self._insert("person", ts=1.0)
        self.store.mark_synced(a)
        self._insert("fire", ts=2.0)  # unsynced — must survive
        removed = self.store.prune_synced_older_than(age_sec=0)
        self.assertEqual(removed, 1)
        # Unsynced row still present.
        self.assertEqual(self.store.counts_by_kind(), {"fire": 1})

    def test_ordering_is_by_ts(self):
        self._insert("person", ts=200.0)
        self._insert("fire",   ts=100.0)  # earlier ts, inserted 2nd
        rows = self.store.unsynced()
        self.assertEqual([r["kind"] for r in rows], ["fire", "person"])


if __name__ == "__main__":
    unittest.main()
