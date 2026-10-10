"""Keep only the latest pending status; filesystem latency stays off callbacks."""
from pathlib import Path
import threading


def atomic_text(path, text):
    temporary = path.with_suffix('.tmp')
    temporary.write_text(text, encoding='utf-8')
    temporary.replace(path)


class AsyncCheckpoint:
    """One immutable pending snapshot plus one in flight. Final status writes last."""
    def __init__(self, path, *, write=atomic_text, max_bytes=1024*1024):
        self.path, self.write, self.max_bytes = Path(path), write, max_bytes
        self.condition = threading.Condition()
        self.pending = None
        self.closing = False
        self.error = None
        self.thread = threading.Thread(target=self._run, name='capture-status', daemon=True)
        self.thread.start()

    def submit(self, text):
        with self.condition:
            if self.error or self.closing:
                return False
            if len(text.encode('utf-8')) > self.max_bytes:
                self.error = 'checkpoint_capacity_exceeded'
                self.condition.notify_all()
                return False
            self.pending = text
            self.condition.notify()
        return True

    def _run(self):
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda: self.pending is not None or self.closing or self.error)
                    if self.error:
                        return
                    text, self.pending = self.pending, None
                    if text is None and self.closing:
                        return
                self.write(self.path, text)
        except Exception as exc:
            self.error = 'checkpoint_write_failed:'+type(exc).__name__

    def close(self, final_text, timeout=5.):
        with self.condition:
            if len(final_text.encode('utf-8')) > self.max_bytes:
                self.error = 'checkpoint_capacity_exceeded'
            self.pending = final_text
            self.closing = True
            self.condition.notify_all()
        self.thread.join(timeout)
        if self.thread.is_alive():
            self.error = 'checkpoint_close_timeout'
        if self.error:
            raise OSError(self.error)
