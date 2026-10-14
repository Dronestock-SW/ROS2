"""Opt-in, memory-only timing probes for isolated B_TF replay.

Never imported by the operational node. Both recording arms use these same
wrappers; timing rows are serialized only after both ROS executors stop.
"""
from collections import defaultdict
import functools
import json
import os
from pathlib import Path
import resource
import threading
import time


def distribution(values):
    values = sorted(values)
    if not values:
        return dict(count=0, median=None, p95=None, p99=None, maximum=None)
    def q(fraction):
        return values[min(len(values)-1, int((len(values)-1)*fraction))]
    return dict(count=len(values), median=q(.5), p95=q(.95), p99=q(.99),
                maximum=values[-1])


class Probe:
    def __init__(self):
        self.rows = []
        self.dropped = 0

    def add(self, kind, start, finish):
        if len(self.rows) < 200_000:
            self.rows.append((kind, start, finish))
        else:
            self.dropped += 1

    def timed(self, function, kind):
        @functools.wraps(function)
        def wrapped(*args, **kwargs):
            start = time.monotonic_ns()
            try:
                return function(*args, **kwargs)
            finally:
                self.add(kind, start, time.monotonic_ns())
        return wrapped


class MeasuredLock:
    """Preserves the existing RLock, including nested acquisition."""
    def __init__(self, lock, probe):
        self.lock, self.probe = lock, probe
        self.local = threading.local()

    def __enter__(self):
        start = time.monotonic_ns()
        self.lock.acquire()
        acquired = time.monotonic_ns()
        if not hasattr(self.local, 'stack'):
            self.local.stack = []
        self.local.stack.append((start, acquired))
        return self

    def __exit__(self, *exc):
        start, acquired = self.local.stack.pop()
        finished = time.monotonic_ns()
        self.lock.release()
        self.probe.add('sensor_lock_wait', start, acquired)
        self.probe.add('sensor_lock_hold', acquired, finished)


def main(output):
    from drone_uwb.integration.ros import btf_node
    probe = Probe()
    node_type = btf_node.BtfNode
    for name in ('raw', 'tof', 'imu', 'freeze_sensors', 'record'):
        setattr(node_type, name, probe.timed(getattr(node_type, name), name))
    original_init = node_type.__init__
    def init(node):
        original_init(node)
        node.sensor_lock = MeasuredLock(node.sensor_lock, probe)
        node.processor.process = probe.timed(node.processor.process, 'process')
    node_type.__init__ = init
    started = time.monotonic_ns()
    before = resource.getrusage(resource.RUSAGE_SELF)
    profile = None
    profile_output = os.environ.get('BTF_REPLAY_PROFILE')
    if profile_output:
        import cProfile
        profile = cProfile.Profile()
        profile.enable()
    try:
        btf_node.main()
    finally:
        if profile:
            profile.disable()
            profile.dump_stats(profile_output)
        after = resource.getrusage(resource.RUSAGE_SELF)
        durations = defaultdict(list)
        for kind, start, finish in probe.rows:
            durations[kind].append((finish-start)/1e6)
        result = dict(schema=1, memory_only_during_replay=True,
            dropped=probe.dropped, wall_s=(time.monotonic_ns()-started)/1e9,
            cpu_user_s=after.ru_utime-before.ru_utime,
            cpu_system_s=after.ru_stime-before.ru_stime,
            max_rss_kib=after.ru_maxrss, block_output=after.ru_oublock-before.ru_oublock,
            duration_ms={k:distribution(v) for k,v in durations.items()})
        Path(output).write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
