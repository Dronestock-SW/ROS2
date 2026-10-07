"""Persist local QR observations; suppress continuous duplicate readings."""
import json
import math
import sqlite3
import time
from datetime import datetime, timezone
from uuid import uuid4

SCHEMA_VERSION = 2

# raw_qr_data holds the QR text exactly as read. item_json holds the parsed and
# normalized form. Both are kept because they answer different questions:
# the server contract asks for the original text, while duplicate detection has
# to ignore whitespace and key order. Neither one can be derived from the other.
_DDL = '''CREATE TABLE qr_observations (
    client_scan_id TEXT PRIMARY KEY,
    session_id     TEXT NOT NULL,
    scanned_at_iso TEXT NOT NULL,
    raw_qr_data    TEXT NOT NULL,
    item_json      TEXT NOT NULL
)'''


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
        self._prepare_schema()

    def _prepare_schema(self):
        """Create the table, or refuse a database written by an older layout.

        Version 1 never stamped user_version, so a pre-raw_qr_data database is
        indistinguishable from a fresh one by the stamp alone. The table has to
        be looked up as well, otherwise CREATE TABLE IF NOT EXISTS would keep
        the old four-column table and every insert would fail later.
        """
        version = self.db.execute('PRAGMA user_version').fetchone()[0]
        exists = self.db.execute(
            "SELECT 1 FROM sqlite_master WHERE type='table' "
            "AND name='qr_observations'").fetchone() is not None
        if not exists:
            if version not in (0, SCHEMA_VERSION):
                raise RuntimeError(
                    f'database schema version {version} is not supported')
            with self.db:
                self.db.execute(_DDL)
                self.db.execute(f'PRAGMA user_version = {SCHEMA_VERSION}')
        elif version != SCHEMA_VERSION:
            raise RuntimeError(
                f'database schema version {version} predates raw_qr_data; '
                'delete the file if it only holds test records')

    def record(self, item, now=None):
        """Return a stored observation, or None for a repeated reading."""
        fields = ('schema', 'code', 'name', 'zone', 'shelf', 'slot', 'location', 'date')
        if not isinstance(item, dict) or item.get('schema') != 'drone-stock-item/v1':
            raise ValueError('invalid QR schema')
        normalized = {key: item.get(key, '') for key in fields}
        if (not all(isinstance(value, str) for value in normalized.values())
                or not normalized['code'] or not normalized['name']):
            raise ValueError('QR fields must be strings; code and name are required')
        raw = item.get('raw_qr_data')
        if not isinstance(raw, str) or not raw:
            raise ValueError('raw_qr_data is required; qr_parser_node supplies it')
        # The duplicate key leaves raw_qr_data out on purpose. Two reads of one
        # label may differ in spacing yet mean the same thing.
        key = json.dumps(normalized, ensure_ascii=False, sort_keys=True)
        now = time.monotonic() if now is None else now
        self.last_seen = {k: t for k, t in self.last_seen.items()
                          if now - t < self.window}
        if key in self.last_seen:
            self.last_seen[key] = now
            return None
        record = dict(client_scan_id=str(uuid4()), session_id=self.session_id,
                      scanned_at_iso=datetime.now(timezone.utc).isoformat(),
                      raw_qr_data=raw, item=normalized)
        # Only mark as seen after the transaction commits successfully.
        # Columns are named so that later additions cannot shift the order.
        with self.db:
            self.db.execute(
                'INSERT INTO qr_observations (client_scan_id, session_id, '
                'scanned_at_iso, raw_qr_data, item_json) VALUES (?, ?, ?, ?, ?)', (
                    record['client_scan_id'], record['session_id'],
                    record['scanned_at_iso'], raw, key))
        self.last_seen[key] = now
        return record

    def close(self):
        self.db.close()
