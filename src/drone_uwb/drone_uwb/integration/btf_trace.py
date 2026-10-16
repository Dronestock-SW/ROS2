"""Bounded rotating B_TF diagnostics. JSON encoding and disk I/O run off ingress.

Callers transfer ownership of rows to write(). Queue exhaustion fails the trace
explicitly; it never blocks observations. Linux advisory lock prevents two nodes
from rotating one another's files. Each segment starts with the run context.
"""
import json
import math
from pathlib import Path
import queue
import shutil
import threading
import time
import uuid


def safe_json(value):
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: safe_json(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [safe_json(v) for v in value]
    return value


class BtfTrace:
    def __init__(self, directory, context, *, segment_bytes=32*1024*1024,
                 segments=4, capacity=4096, min_free_bytes=256*1024*1024):
        if min(segment_bytes, segments, capacity) <= 0:
            raise ValueError('positive_trace_capacity_required')
        self.directory = Path(directory)
        self.segment_bytes, self.segments = segment_bytes, segments
        self.min_free_bytes = min_free_bytes
        self.context = dict(type='context', schema=1, session=uuid.uuid4().hex,
                            started_wall_ns=time.time_ns(), **context)
        self.pending = queue.Queue(maxsize=capacity)
        self.error = None
        self.dropped = 0
        self.written = 0
        self.closing = threading.Event()
        self.thread = threading.Thread(target=self._run, name='btf-trace', daemon=True)
        self.thread.start()

    def write(self, stream, row, *, encoded_row=None):
        """Accept an optional immutable JSON row already encoded by its publisher.

        encoded_row must come from the same strict safe_json/json.dumps boundary
        as the published message. Keeping it avoids a second recursive conversion
        on the writer thread, which shares the sensor callbacks' Python GIL.
        """
        if self.error or self.closing.is_set():
            self.dropped += 1
            return False
        try:
            self.pending.put_nowait(dict(stream=stream, row=row if encoded_row is None else None,
                encoded_row=encoded_row, enqueued_monotonic_ns=time.monotonic_ns()))
            return True
        except queue.Full:
            self.dropped += 1
            self.error = 'trace_queue_full'
            return False

    def _path(self, index):
        return self.directory / ('trace.jsonl' if index == 0 else f'trace.{index}.jsonl')

    def _rotate(self):
        self._path(self.segments-1).unlink(missing_ok=True)
        for i in range(self.segments-2, -1, -1):
            if self._path(i).exists():
                self._path(i).replace(self._path(i+1))

    def _run(self):
        stream = lock_file = None
        try:
            import fcntl
            self.directory.mkdir(parents=True, exist_ok=True)
            lock_file = (self.directory/'.writer.lock').open('a', encoding='utf-8')
            fcntl.flock(lock_file, fcntl.LOCK_EX | fcntl.LOCK_NB)
            header = (json.dumps(safe_json(self.context), allow_nan=False)+'\n').encode('utf-8')
            if len(header) >= self.segment_bytes:
                raise ValueError('trace_context_exceeds_segment')
            # Start a fresh session without truncating the previous current file.
            self._rotate()
            size, checked_bytes, last_flush = 0, 1024*1024, time.monotonic()
            while not self.closing.is_set() or not self.pending.empty():
                try:
                    item = self.pending.get(timeout=.1)
                except queue.Empty:
                    if stream:
                        stream.flush()
                    if self.error:
                        break
                    continue
                encoded = item.pop('encoded_row')
                if encoded is None:
                    line = json.dumps(safe_json(item), ensure_ascii=False, allow_nan=False)
                else:
                    # Only the small envelope is new. Do not reparse or walk
                    # the already validated row; the on-disk schema is unchanged.
                    item.pop('row')
                    line = json.dumps(item, ensure_ascii=False, allow_nan=False)[:-1]+', "row": '+encoded+'}'
                data = (line+'\n').encode('utf-8')
                if len(data)+len(header) > self.segment_bytes:
                    raise ValueError('trace_row_exceeds_segment')
                if checked_bytes >= 1024*1024:
                    if shutil.disk_usage(self.directory).free < self.min_free_bytes:
                        raise OSError('trace_disk_reserve_low')
                    checked_bytes = 0
                if stream and size+len(data) > self.segment_bytes:
                    stream.close()
                    stream = None
                    self._rotate()
                if stream is None:
                    stream = self._path(0).open('xb')
                    stream.write(header)
                    size = len(header)
                stream.write(data)
                size += len(data)
                checked_bytes += len(data)
                self.written += 1
                if time.monotonic()-last_flush >= 1.:
                    stream.flush()
                    last_flush = time.monotonic()
        except Exception as exc:
            self.error = self.error or f'trace_write_failed:{type(exc).__name__}:{exc}'
        finally:
            if stream:
                try:
                    stream.close()
                except OSError as exc:
                    self.error = self.error or f'trace_close_failed:{exc}'
            if lock_file:
                lock_file.close()

    def status(self):
        return dict(directory=str(self.directory), error=self.error, dropped=self.dropped,
                    pending=self.pending.qsize(), written=self.written,
                    max_disk_bytes=self.segment_bytes*self.segments,
                    session=self.context['session'])

    def close(self, timeout=5.):
        self.closing.set()
        self.thread.join(timeout)
        if self.thread.is_alive():
            self.error = self.error or 'trace_close_timeout'
