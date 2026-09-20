"""Persist local QR observations; suppress continuous duplicate readings."""
import json
import math
import sqlite3
import time
from datetime import datetime, timezone
from uuid import uuid4


class QrRecordStore:
    """Each process session is independent of mission-level counting policy."""

    def __init__(self, path, duplicate_window_sec=3.0):
        if not math.isfinite(duplicate_window_sec) or duplicate_window_sec <= 0:
            raise ValueError('duplicate_window_sec must be positive and finite')
        self.window = duplicate_window_sec
        self.session_id = str(uuid4())
        self.last_seen = {}
        self.db = sqlite3.connect(path)
        self.db.execute('PRAGMA synchronous=FULL')
        self.db.execute('''CREATE TABLE IF NOT EXISTS qr_observations (
            client_scan_id TEXT PRIMARY KEY,
            session_id TEXT NOT NULL,
            scanned_at_iso TEXT NOT NULL,
            item_json TEXT NOT NULL
        )''')
        self.db.commit()

    def record(self, item, now=None):
        """Return a stored observation, or None for a repeated reading."""
        fields = ('schema', 'code', 'name', 'zone', 'shelf', 'slot', 'location', 'date')
        if not isinstance(item, dict) or item.get('schema') != 'drone-stock-item/v1':
            raise ValueError('invalid QR schema')
        normalized = {key: item.get(key, '') for key in fields}
        if (not all(isinstance(value, str) for value in normalized.values())
                or not normalized['code'] or not normalized['name']):
            raise ValueError('QR fields must be strings; code and name are required')
        key = json.dumps(normalized, ensure_ascii=False, sort_keys=True)
        now = time.monotonic() if now is None else now
        self.last_seen = {k: t for k, t in self.last_seen.items()
                          if now - t < self.window}
        if key in self.last_seen:
            self.last_seen[key] = now
            return None
        record = dict(client_scan_id=str(uuid4()), session_id=self.session_id,
                      scanned_at_iso=datetime.now(timezone.utc).isoformat(),
                      item=normalized)
        # Only mark as seen after the transaction commits successfully.
        with self.db:
            self.db.execute('INSERT INTO qr_observations VALUES (?, ?, ?, ?)', (
                record['client_scan_id'], record['session_id'],
                record['scanned_at_iso'], key))
        self.last_seen[key] = now
        return record

    def close(self):
        self.db.close()
