"""Storage stalls must not stall capture or overwrite a final summary."""
import json
import threading
import time

import pytest

from drone_uwb.integration.async_checkpoint import AsyncCheckpoint, atomic_text
from drone_uwb.integration.manual_capture import Capture


def test_slow_checkpoint_coalesces_and_final_status_writes_last(tmp_path):
    entered, release = threading.Event(), threading.Event()
    writes = []
    def slow(path, text):
        entered.set()
        assert release.wait(3)
        writes.append(text)
        atomic_text(path, text)
    path = tmp_path/'summary.json'
    sink = AsyncCheckpoint(path, write=slow)
    sink.submit('first')
    assert entered.wait(1)
    try:
        start = time.monotonic()
        for n in range(1000):
            assert sink.submit(str(n))
        assert time.monotonic()-start < .5
        assert sink.pending == '999'
        # Final snapshot replaces pending intermediate status even while I/O is blocked.
        closer = threading.Thread(target=lambda:sink.close('final'))
        closer.start()
        deadline = time.monotonic()+1
        while not sink.closing and time.monotonic()<deadline:
            time.sleep(.001)
        assert sink.closing and not sink.submit('too late')
    finally:
        release.set()
    closer.join(2)
    assert not closer.is_alive()
    assert writes == ['first','final'] and path.read_text() == 'final'


def test_checkpoint_failure_is_latched(tmp_path):
    def fail(*args): raise OSError('disk full')
    sink = AsyncCheckpoint(tmp_path/'status',write=fail)
    sink.submit('status')
    with pytest.raises(OSError,match='checkpoint_write_failed:OSError'):
        sink.close('final')
    assert not sink.submit('later')


def test_checkpoint_close_timeout_discards_pending_final_status(tmp_path):
    entered, release = threading.Event(), threading.Event()
    writes = []
    def slow(*args):
        entered.set(); release.wait(3); writes.append(args[1])
    sink = AsyncCheckpoint(tmp_path/'status',write=slow)
    sink.submit('recording')
    assert entered.wait(1)
    try:
        with pytest.raises(OSError,match='checkpoint_close_timeout'):
            sink.close('success',timeout=.01)
    finally:
        release.set(); sink.thread.join(1)
    assert writes == ['recording']


def test_capture_continues_during_checkpoint_stall_and_preserves_snapshot(tmp_path):
    c = Capture(tmp_path/'capture',{},min_free_bytes=0)
    entered, release = threading.Event(), threading.Event()
    def slow(path, text):
        entered.set(); assert release.wait(3); atomic_text(path,text)
    c.checkpoints.write = slow
    c.checkpoint()
    assert entered.wait(1)
    try:
        start = time.monotonic()
        for n in range(100):
            assert c.add('sensor','test',{'n':n},mono_ns=n,ros_ns=n)
        c.checkpoint()
        assert time.monotonic()-start < .5
        assert json.loads(c.checkpoints.pending)['topics']['sensor']['queued'] == 100
        assert c.add('sensor','test',{},mono_ns=101,ros_ns=101)
        assert json.loads(c.checkpoints.pending)['topics']['sensor']['queued'] == 100
    finally:
        release.set()
    result = c.close('test')
    assert result['writer_drained'] and result['topics']['sensor']['queued'] == 101
    assert json.loads((c.directory/'summary.json').read_text()) == result


def test_status_worker_polls_stop_file_and_failure_rejects_further_events(tmp_path):
    c = Capture(tmp_path/'capture',{},min_free_bytes=0)
    (c.directory/'STOP').touch()
    c.checkpoint()
    deadline=time.monotonic()+1
    while not c.stop_requested and time.monotonic()<deadline:
        time.sleep(.001)
    assert c.stop_requested
    c.close('operator_stop_file')
    c = Capture(tmp_path/'failed',{},min_free_bytes=0)
    failed=threading.Event()
    def fail(*args):
        failed.set(); raise OSError('checkpoint fault')
    c.checkpoints.write=fail
    c.checkpoint()
    assert failed.wait(1)
    c.checkpoints.thread.join(1)
    assert not c.add('sensor','test',{},mono_ns=1,ros_ns=1)
    result=c.close('recording_error')
    assert result['error']=='checkpoint_write_failed:OSError' and not result['writer_drained']
    assert not json.loads((c.directory/'summary.json').read_text())['stopped']
