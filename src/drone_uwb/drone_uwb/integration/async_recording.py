"""Bounded FIFO recording; storage latency must not block sensor callbacks."""
from collections import deque
import threading


class AsyncRecording:
    def __init__(self, streams, max_pending_bytes=16*1024*1024, max_pending_items=8192):
        if max_pending_bytes <= 0 or max_pending_items <= 0:
            raise ValueError('positive_recording_capacity_required')
        self.streams = streams
        self.max_bytes, self.max_items = max_pending_bytes, max_pending_items
        self.pending = deque()
        self.pending_bytes = 0
        self.error = None
        self.condition = threading.Condition()
        self.closing = False
        self.flush_pending = False
        self.thread = threading.Thread(target=self._run, name='uwb-recording', daemon=True)
        self.thread.start()

    def write(self, name, data):
        size = len(data) if isinstance(data, bytes) else len(data.encode('utf-8'))
        with self.condition:
            if self.error or self.closing:
                return False
            if self.pending_bytes+size > self.max_bytes or len(self.pending) >= self.max_items:
                self.error = 'recording_queue_full'
                self.condition.notify_all()
                return False
            self.pending.append((name, data, size))
            self.pending_bytes += size
            self.condition.notify()
        return True

    def flush(self):
        with self.condition:
            if not self.flush_pending and not self.error and not self.closing:
                self.pending.append((None, None, 0))
                self.flush_pending = True
                self.condition.notify()

    def _run(self):
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.pending or self.closing or self.error)
                    if not self.pending:
                        if self.closing or self.error:
                            break
                        continue
                    name, data, size = self.pending.popleft()
                    # In-flight data remains charged until the write completes.
                if name is None:
                    for stream in self.streams.values():
                        stream.flush()
                else:
                    self.streams[name].write(data)
                with self.condition:
                    self.pending_bytes -= size
                    if name is None:
                        self.flush_pending = False
        except Exception as exc:
            self.error = 'recording_write_failed:'+type(exc).__name__
        finally:
            for stream in self.streams.values():
                try:
                    stream.close()
                except Exception as exc:
                    self.error = self.error or 'recording_close_failed:'+type(exc).__name__

    def close(self, timeout=5.):
        with self.condition:
            self.closing = True
            self.condition.notify_all()
        self.thread.join(timeout)
        if self.thread.is_alive():
            self.error = 'recording_close_timeout'
            raise OSError(self.error)
        if self.error:
            raise OSError(self.error)
