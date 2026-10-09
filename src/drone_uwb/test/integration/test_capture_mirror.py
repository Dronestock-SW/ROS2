"""Snapshot protocol faults must preserve the local evidence prefix."""
import importlib.util
import io
import json
from pathlib import Path
import sys
from types import SimpleNamespace

import pytest

SPEC = importlib.util.spec_from_file_location('mirror', Path(__file__).parents[2]/'tools/mirror_manual_capture.py')
mirror = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(mirror)


def source(tmp_path):
    root = tmp_path/'source'; root.mkdir()
    (root/'manifest.json').write_text(json.dumps(dict(boot_id='boot', started={'monotonic_ns':1},
        process={'pid':222, 'start_ticks':'111'})))
    (root/'summary.json').write_text(json.dumps(dict(stopped=True, error=None)))
    (root/'events.jsonl').write_bytes(b'{"x":1}\n'*100)
    return root


def snapshot(monkeypatch, root, offset=0, *, boot='boot', ticks='111', age=0):
    # Execute the exact remote script against real files; fake only /proc and
    # the active process clock so tests work on Windows and Linux alike.
    original = Path.read_text
    def read_text(path, *args, **kwargs):
        if str(path).replace('\\', '/') == '/proc/sys/kernel/random/boot_id':
            return boot
        if str(path).replace('\\', '/') == '/proc/222/stat':
            return '222 (name with ) spaces) '+' '.join(['S']+['0']*18+[ticks])
        return original(path, *args, **kwargs)
    with monkeypatch.context() as patch:
        buffer = io.BytesIO()
        patch.setattr(Path, 'read_text', read_text)
        patch.setattr(sys, 'argv', ['snapshot', str(root), str(offset), '222'])
        patch.setattr(sys, 'stdout', SimpleNamespace(buffer=buffer))
        patch.setattr(mirror.time, 'monotonic_ns', lambda: int((100+age)*1e9))
        exec(mirror.REMOTE_SNAPSHOT, {})
        return buffer.getvalue()


def test_resume_updates_metadata_and_full_hash(monkeypatch, tmp_path):
    root = source(tmp_path); out = tmp_path/'out'; out.mkdir()
    info = mirror.accept_snapshot(out, snapshot(monkeypatch, root))
    assert mirror.digest(out/'events.jsonl') == info['sha256']
    with (root/'events.jsonl').open('ab') as f: f.write(b'{"x":2}\n')
    info = mirror.accept_snapshot(out, snapshot(monkeypatch, root, 800))
    assert info['count'] == 8 and mirror.digest(out/'events.jsonl') == info['sha256']


@pytest.mark.parametrize('fault', ['partial', 'changed_boot', 'changed_prefix', 'shrunk', 'missing'])
def test_resume_fault_never_appends_or_reports_success(monkeypatch, tmp_path, fault):
    root = source(tmp_path); out = tmp_path/'out'; out.mkdir()
    mirror.accept_snapshot(out, snapshot(monkeypatch, root))
    before = (out/'events.jsonl').read_bytes()
    if fault == 'changed_boot':
        m = json.loads((root/'manifest.json').read_text()); m['boot_id']='different'
        (root/'manifest.json').write_text(json.dumps(m))
    if fault == 'changed_prefix': (root/'events.jsonl').write_bytes(b'z'*800)
    if fault == 'shrunk': (root/'events.jsonl').write_bytes(b'z')
    if fault == 'missing': (root/'manifest.json').unlink()
    payload = snapshot(monkeypatch, root, len(before))
    if fault == 'partial': payload = payload[:-3]
    with pytest.raises(ValueError): mirror.accept_snapshot(out, payload)
    assert (out/'events.jsonl').read_bytes() == before


@pytest.mark.parametrize('kwargs,error', [({'boot':'new'}, 'source_host_rebooted'),
    ({'ticks':'999'}, 'source_pid_reused'), ({'age':16}, 'source_checkpoint_stale')])
def test_live_source_identity_and_checkpoint(monkeypatch, tmp_path, kwargs, error):
    root = source(tmp_path); out = tmp_path/'out'; out.mkdir()
    (root/'summary.json').write_text(json.dumps(dict(stopped=False, updated={'monotonic_ns':100_000_000_000})))
    with pytest.raises(mirror.SourceError, match=error):
        mirror.accept_snapshot(out, snapshot(monkeypatch, root, **kwargs))
    assert not (out/'events.jsonl').exists()


def test_main_records_failure_without_retrying_missing_source(monkeypatch, tmp_path):
    calls=[]
    def run(*args, **kwargs):
        calls.append(kwargs)
        return SimpleNamespace(returncode=0, stdout=b'{"error":"source_missing"}\n')
    monkeypatch.setattr(mirror.subprocess, 'run', run)
    assert mirror.main(['--host','host','--remote-dir','/dev/shm/old','--pid','222',
                        '--output',str(tmp_path)]) == 1
    status=json.loads((tmp_path/'mirror-status.json').read_text())
    assert len(calls)==1 and calls[0]['timeout']==30
    assert not status['complete'] and status['last_error']=='source_missing'
