"""Durable local mission drafts, single-run reservation and request receipts.

This store cannot send aircraft commands or reset the companion flight session.
"""
import hashlib
import json
from pathlib import Path
import sqlite3
import time
import uuid


def encoded(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), allow_nan=False, ensure_ascii=False)


def digest(value):
    return hashlib.sha256(encoded(value).encode('utf-8')).hexdigest()


class MissionStore:
    def __init__(self, path, drone_id, layout_id):
        if path:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
        self.db = sqlite3.connect(str(path) if path else ':memory:', check_same_thread=False)
        self.db.row_factory = sqlite3.Row
        self.db.executescript('''
            PRAGMA synchronous=FULL;
            CREATE TABLE IF NOT EXISTS identity (id INTEGER PRIMARY KEY CHECK(id=1), body TEXT NOT NULL);
            CREATE TABLE IF NOT EXISTS drafts (
                id TEXT PRIMARY KEY, name TEXT NOT NULL, name_key TEXT NOT NULL,
                revision INTEGER NOT NULL, body TEXT NOT NULL, digest TEXT NOT NULL,
                archived INTEGER NOT NULL DEFAULT 0, updated REAL NOT NULL);
            CREATE UNIQUE INDEX IF NOT EXISTS draft_name ON drafts(name_key) WHERE archived=0;
            CREATE UNIQUE INDEX IF NOT EXISTS draft_body ON drafts(digest) WHERE archived=0;
            CREATE TABLE IF NOT EXISTS runs (
                id INTEGER PRIMARY KEY AUTOINCREMENT, draft_id TEXT, name TEXT NOT NULL,
                assignment TEXT NOT NULL, open INTEGER NOT NULL DEFAULT 1,
                state TEXT NOT NULL, result TEXT NOT NULL DEFAULT '{}', created REAL NOT NULL);
            CREATE UNIQUE INDEX IF NOT EXISTS one_open_run ON runs(open) WHERE open=1;
            CREATE TABLE IF NOT EXISTS receipts (id TEXT PRIMARY KEY, digest TEXT NOT NULL, response TEXT NOT NULL);
        ''')
        identity = encoded(dict(drone_id=drone_id, layout_id=layout_id, schema=1))
        with self.db:
            self.db.execute('INSERT OR IGNORE INTO identity VALUES(1,?)', (identity,))
        if self.db.execute('SELECT body FROM identity').fetchone()[0] != identity:
            raise ValueError('미션 저장소의 기체·앵커 배치가 다릅니다.')

    def listing(self):
        drafts = [dict(r) for r in self.db.execute('SELECT * FROM drafts WHERE archived=0 ORDER BY updated DESC')]
        runs = [dict(r) for r in self.db.execute('SELECT * FROM runs ORDER BY id DESC LIMIT 100')]
        for row in drafts:
            row['definition'] = json.loads(row.pop('body'))
        for row in runs:
            row['result'] = json.loads(row['result'])
            row['assignment'] = json.loads(row['assignment'])
        return dict(drafts=drafts, runs=runs)

    def draft(self, key):
        row = self.db.execute('SELECT * FROM drafts WHERE id=? AND archived=0', (key,)).fetchone()
        if row is None:
            raise ValueError('저장 미션을 찾을 수 없습니다.')
        return dict(row, definition=json.loads(row['body']))

    def save_draft(self, name, definition, key=None, revision=None):
        if not isinstance(name, str) or not 1 <= len(name.strip()) <= 80:
            raise ValueError('미션 이름은 1~80자로 입력하세요.')
        name = name.strip()
        fingerprint = digest(definition)
        same = self.db.execute('SELECT id FROM drafts WHERE digest=? AND archived=0', (fingerprint,)).fetchone()
        if same and same['id'] != key:
            return dict(ok=True, duplicate=True, draft_id=same['id'], flight_command_sent=False)
        if key:
            self.draft(key)
            if self.db.execute('SELECT 1 FROM runs WHERE draft_id=? AND open=1', (key,)).fetchone():
                raise ValueError('실행 중인 미션 원본은 수정할 수 없습니다.')
        try:
            with self.db:
                if key:
                    if type(revision) is not int:
                        raise ValueError('미션 버전이 필요합니다.')
                    cursor = self.db.execute('UPDATE drafts SET name=?,name_key=?,body=?,digest=?,revision=revision+1,updated=? WHERE id=? AND revision=? AND archived=0',
                        (name, name.casefold(), encoded(definition), fingerprint, time.time(), key, revision))
                    if cursor.rowcount != 1:
                        raise ValueError('다른 창에서 수정했습니다. 목록을 다시 불러오세요.')
                else:
                    key = uuid.uuid4().hex
                    self.db.execute('INSERT INTO drafts VALUES(?,?,?,?,?,?,0,?)',
                        (key, name, name.casefold(), 1, encoded(definition), fingerprint, time.time()))
        except sqlite3.IntegrityError:
            raise ValueError('같은 이름의 미션이 있습니다.') from None
        return dict(ok=True, duplicate=False, draft_id=key, flight_command_sent=False)

    def archive_draft(self, key, revision):
        if self.db.execute('SELECT 1 FROM runs WHERE draft_id=? AND open=1', (key,)).fetchone():
            raise ValueError('실행 중인 미션은 삭제할 수 없습니다.')
        with self.db:
            cursor = self.db.execute('UPDATE drafts SET archived=1 WHERE id=? AND revision=? AND archived=0', (key, revision))
            if cursor.rowcount != 1:
                raise ValueError('미션 버전이 바뀌었습니다. 목록을 다시 불러오세요.')
        return dict(ok=True, flight_command_sent=False)

    def receipt(self, key, body):
        if not isinstance(key, str) or not 8 <= len(key) <= 128:
            raise ValueError('요청 식별자가 필요합니다.')
        row = self.db.execute('SELECT * FROM receipts WHERE id=?', (key,)).fetchone()
        if row:
            if row['digest'] != digest(body):
                raise ValueError('같은 요청 식별자로 다른 명령을 보낼 수 없습니다.')
            return json.loads(row['response'])

    def active(self):
        row = self.db.execute('SELECT * FROM runs WHERE open=1').fetchone()
        return dict(row) if row else None

    def start(self, key, body, assignment, name, draft_id=None):
        # Reserve durably before exposing a START to the polling companion.
        try:
            with self.db:
                cursor = self.db.execute('INSERT INTO runs(draft_id,name,assignment,state,created) VALUES(?,?,?,?,?)',
                    (draft_id, name, '{}', 'REQUESTED', time.time()))
                run_id = cursor.lastrowid
                assignment = dict(assignment, mission_db_id=run_id, mission_code='LOCAL-'+uuid.uuid4().hex[:12])
                self.db.execute('UPDATE runs SET assignment=? WHERE id=?', (encoded(assignment), run_id))
                self.db.execute('INSERT INTO receipts VALUES(?,?,?)', (key, digest(body), encoded(assignment)))
        except sqlite3.IntegrityError:
            raise ValueError('진행 중이거나 결과 미확인인 미션이 있습니다.') from None
        return assignment

    def control(self, key, body, assignment):
        with self.db:
            self.db.execute('UPDATE runs SET assignment=? WHERE id=? AND open=1',
                            (encoded(assignment), assignment['mission_db_id']))
            self.db.execute('INSERT INTO receipts VALUES(?,?,?)', (key, digest(body), encoded(assignment)))

    def acknowledge(self, assignment):
        with self.db:
            self.db.execute('UPDATE runs SET assignment=? WHERE id=?',
                            (encoded(assignment), assignment.get('mission_db_id')))

    def observe(self, telemetry):
        row = self.active()
        if not row:
            return
        assignment = json.loads(row['assignment'])
        if any(telemetry.get(k) != assignment.get(k) for k in ('mission_db_id', 'mission_code', 'route_revision')):
            return
        keys = ('flight_state', 'flight_reason', 'flight_outcome', 'work_outcome', 'landing_verified',
                'home_verified', 'mission_complete', 'route_complete', 'scan_results', 'failed_scan_task_ids')
        result = {k: telemetry.get(k) for k in keys}
        text = encoded(result)
        if text != row['result']:
            with self.db:
                self.db.execute('UPDATE runs SET state=?,result=? WHERE id=?',
                    (telemetry.get('flight_state') or 'UNKNOWN', text, row['id']))

    def close_run(self, key):
        with self.db:
            self.db.execute('UPDATE runs SET open=0 WHERE id=?', (key,))

    def close(self):
        self.db.close()
