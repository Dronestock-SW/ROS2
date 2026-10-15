"""Instance-owned instantaneous XY geometry, rebuilt on process restart.

The caller's anchor configuration and selected indices are fixed for this
processor lifetime. Every solve still uses the current measured ranges.
"""
import hashlib
import json
import time
import numpy as np
from drone_uwb.contracts.protocol import InvalidSample


class CachedXY:
    def __init__(self, anchors, indices=(0, 1, 2, 3)):
        started = time.perf_counter_ns()
        cpu_started = time.thread_time_ns()
        anchors = np.array(anchors, dtype=float, copy=True)
        self.base = indices[0]
        self.others = np.asarray(indices[1:], dtype=int)
        offsets = anchors[self.others, :2] - anchors[self.base, :2]
        self.matrix = 2.0 * offsets
        self.constant = (np.sum(anchors[self.others, :2] ** 2, axis=1)
                         - np.sum(anchors[self.base, :2] ** 2))
        self.denominator = 2 * np.linalg.norm(offsets, axis=1)
        self.inverse, _, rank, singular = np.linalg.lstsq(
            self.matrix, np.eye(len(self.others)), rcond=None)
        self.usable = bool(rank == 2 and singular[0] / singular[-1] <= 30)
        for array in (self.others, self.matrix, self.constant,
                      self.denominator, self.inverse):
            array.flags.writeable = False
        identity = dict(anchors_xyz_m=anchors.tolist(), indices=list(indices))
        self.geometry_sha256 = hashlib.sha256(
            json.dumps(identity, sort_keys=True).encode('utf-8')).hexdigest()
        self.builds = 1
        self.calls = 0
        self.build_cpu_ms = (time.thread_time_ns() - cpu_started) / 1e6
        self.build_wall_ms = (time.perf_counter_ns() - started) / 1e6

    def solve(self, ranges):
        self.calls += 1
        rhs = (ranges[self.base] ** 2 - ranges[self.others] ** 2
               + self.constant)
        if not self.usable:
            raise InvalidSample('degenerate_geometry')
        xy = self.inverse @ rhs
        residual = float(np.max(np.abs(rhs - self.matrix @ xy) / self.denominator))
        return xy, residual

    def audit(self):
        return dict(builds=self.builds, calls=self.calls,
                    geometry_sha256=self.geometry_sha256, usable=self.usable,
                    build_wall_ms=self.build_wall_ms, build_cpu_ms=self.build_cpu_ms)
