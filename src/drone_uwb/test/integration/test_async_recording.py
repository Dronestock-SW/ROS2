import io
import threading
import time
import pytest
from drone_uwb.integration.async_recording import AsyncRecording


class SlowFile(io.StringIO):
    def __init__(self):
        super().__init__(); self.entered=threading.Event(); self.release=threading.Event(); self.saved=None

    def write(self, value):
        self.entered.set()
        assert self.release.wait(3.)
        return super().write(value)

    def close(self):
        self.saved=self.getvalue()
        super().close()


def test_slow_storage_does_not_block_callbacks_and_close_drains_fifo():
    stream=SlowFile();sink=AsyncRecording({'raw':stream})
    assert sink.write('raw','first\n') and stream.entered.wait(1.)
    started=time.monotonic()
    try:
        for n in range(100):
            assert sink.write('raw',f'{n}\n')
            sink.flush()
        assert time.monotonic()-started < .5
        assert len(sink.pending)<=101  # Coalesce repeated flushes.
    finally:
        stream.release.set();sink.close()
    assert stream.saved=='first\n'+''.join(f'{n}\n' for n in range(100))


def test_capacity_includes_blocked_write_and_failure_latches():
    stream=SlowFile();sink=AsyncRecording({'raw':stream},max_pending_bytes=10)
    assert sink.write('raw','0123456789') and stream.entered.wait(1.)
    try:
        assert not sink.write('raw','a')
        assert sink.error=='recording_queue_full'
        assert not sink.write('raw','')
    finally:
        stream.release.set()
        with pytest.raises(OSError,match='recording_queue_full'):sink.close()
    assert stream.saved=='0123456789'


def test_disk_error_is_reported_and_rejects_further_recording():
    class Failed(io.StringIO):
        def write(self, data):raise OSError('disk full')
    sink=AsyncRecording({'raw':Failed()})
    assert sink.write('raw','source data')
    with pytest.raises(OSError,match='recording_write_failed:OSError'):sink.close()
    assert not sink.write('raw','more')
