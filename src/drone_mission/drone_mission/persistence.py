"""Bounded disk workers. Their futures never run on sensor callbacks."""
from concurrent.futures import Future
import os
import json
from pathlib import Path
import threading
import time


class DiskWorker:
    def __init__(self, name, capacity=128):
        self.name, self.capacity = name, capacity
        self.condition = threading.Condition()
        self.jobs = {}
        self.sequence = 0
        self.thread = None
        self.stopping = False
        self.error = None
        self.running_since = None
        self.running_key = None
        self.last_elapsed_s = self.max_elapsed_s = 0.
        self.completed = 0

    def submit(self, key, function, *, priority=1, replace=False):
        with self.condition:
            if key in self.jobs:
                previous = self.jobs[key]
                if replace:
                    self.jobs[key] = (priority, previous[1], function, previous[3])
                return previous[3]
            future = Future()
            if self.stopping or len(self.jobs) >= self.capacity:
                future.set_exception(RuntimeError('disk_worker_closed_or_full'))
                self.error = 'disk_worker_closed_or_full'
                return future
            self.sequence += 1
            self.jobs[key] = (priority, self.sequence, function, future)
            if self.thread is None:
                self.thread = threading.Thread(target=self._run, name=self.name, daemon=True)
                self.thread.start()
            self.condition.notify()
            return future

    def _run(self):
        while True:
            with self.condition:
                self.condition.wait_for(lambda: self.jobs or self.stopping)
                if not self.jobs:
                    return
                key = min(self.jobs, key=lambda k: self.jobs[k][:2])
                _, _, function, future = self.jobs.pop(key)
                self.running_key, self.running_since = key, time.monotonic()
            try:
                future.set_result(function())
            except Exception as exc:
                self.error = type(exc).__name__ + ':' + str(exc)
                future.set_exception(exc)
            finally:
                with self.condition:
                    self.last_elapsed_s = time.monotonic()-self.running_since
                    self.max_elapsed_s = max(self.max_elapsed_s, self.last_elapsed_s)
                    self.completed += 1
                    self.running_key = self.running_since = None

    def status(self):
        with self.condition:
            return dict(pending=len(self.jobs), running=self.running_key,
                        running_age_s=0. if self.running_since is None else time.monotonic()-self.running_since,
                        last_elapsed_s=self.last_elapsed_s, max_elapsed_s=self.max_elapsed_s,
                        completed=self.completed, error=self.error)

    def close(self, timeout=3.):
        with self.condition:
            self.stopping = True
            self.condition.notify_all()
        if self.thread:
            self.thread.join(timeout)
        return self.thread is None or not self.thread.is_alive()


def write_consumed(path, consumed):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_suffix('.tmp')
    with temporary.open('w', encoding='utf-8') as stream:
        json.dump({'consumed': sorted(consumed)}, stream)
        stream.flush()
        os.fsync(stream.fileno())
    temporary.replace(path)
    # Directory durability is available on the Linux flight host.
    if hasattr(os, 'O_DIRECTORY'):
        descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)


class RequestJournal:
    def __init__(self, path, consumed=()):
        self.path = Path(path)
        self.persisted = set(consumed)
        self.worker = DiskWorker('request-journal')
        self.requests = {}

    def submit(self, key):
        if key not in self.requests:
            def write():
                updated = self.persisted | {key}
                write_consumed(self.path, updated)
                self.persisted = updated
                return key
            self.requests[key] = self.worker.submit(key, write)
        return self.requests[key]

