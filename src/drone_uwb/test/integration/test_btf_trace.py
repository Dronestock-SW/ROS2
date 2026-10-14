import json
import time
import threading
from types import SimpleNamespace

import pytest

pytest.importorskip('fcntl')
from drone_uwb.integration.btf_trace import BtfTrace


def test_rotating_trace_is_bounded_and_preserves_session_context(tmp_path):
    recorder = BtfTrace(tmp_path, {'config':{'test':True}}, segment_bytes=1024,
                        segments=3, min_free_bytes=0)
    for i in range(100):
        assert recorder.write('inputs', dict(receive_id=i, value=float('nan')))
    recorder.close()
    assert recorder.error is None and recorder.written == 100
    files = list(tmp_path.glob('*.jsonl'))
    assert len(files) == 3 and all(p.stat().st_size <= 1024 for p in files)
    for path in files:
        lines = [json.loads(s) for s in path.read_text().splitlines()]
        assert lines[0]['type'] == 'context'
        assert lines[0]['session'] == recorder.context['session']
        assert all(r['row']['value'] is None for r in lines[1:])
    assert json.loads((tmp_path/'trace.jsonl').read_text().splitlines()[-1])['row']['receive_id'] == 99


def test_full_trace_queue_returns_without_waiting_for_writer(tmp_path, monkeypatch):
    entered, release = threading.Event(), threading.Event()
    def stalled(writer):
        entered.set()
        release.wait(2)
    monkeypatch.setattr(BtfTrace, '_run', stalled)
    writer = BtfTrace(tmp_path, {}, capacity=1)
    assert entered.wait(1)
    try:
        assert writer.write('inputs', {'id':1})
        start = time.monotonic()
        assert not writer.write('inputs', {'id':2})
        assert time.monotonic()-start < .1
        assert writer.error == 'trace_queue_full' and writer.dropped == 1
    finally:
        release.set()
        writer.close()


def test_disk_failure_is_reported_without_raising_in_ingress(tmp_path, monkeypatch):
    monkeypatch.setattr('drone_uwb.integration.btf_trace.shutil.disk_usage',
                        lambda p: SimpleNamespace(free=0))
    writer = BtfTrace(tmp_path, {}, min_free_bytes=1)
    writer.write('inputs', {'id':1})
    writer.close()
    assert 'trace_disk_reserve_low' in writer.error
    assert not writer.write('inputs', {'id':2})


def test_second_writer_cannot_rotate_first_writers_files(tmp_path):
    first = BtfTrace(tmp_path, {}, min_free_bytes=0)
    first.write('inputs', {'id':1})
    deadline = time.monotonic()+2
    while not first.written and not first.error and time.monotonic()<deadline:
        time.sleep(.01)
    try:
        second = BtfTrace(tmp_path, {}, min_free_bytes=0)
        second.write('inputs', {'id':2})
        second.close()
        assert 'BlockingIOError' in second.error
    finally:
        first.close()
    assert first.error is None
    assert not (tmp_path/'trace.1.jsonl').exists()
