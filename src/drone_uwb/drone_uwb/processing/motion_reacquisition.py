"""Bounded measurement requalification, never a pose/velocity publisher.

Only accepted UWB measurements train the motion reference. During quarantine
the reference is frozen; a contradictory candidate latches it closed. PX4,
flow, IMU integration and commanded motion are not inputs to this checker.
"""
from collections import deque
import math

import numpy as np


class MotionReacquisition:
    # Observation-model limits, not certified vehicle operating limits.
    window_s = .5
    min_span_s = .3
    max_gap_s = .1
    horizon_s = 1.5
    min_speed_m_s = .15
    max_speed_m_s = 2.
    fit_error_m = .04
    velocity_error_m_s = .25
    confirm_s = .15
    min_samples = 4

    def __init__(self):
        self.clear()

    def clear(self):
        self.history = deque()
        self.model = None
        self.blocked = None
        self.last_check = None
        self.clear_confirmation()

    def clear_confirmation(self):
        self.pending = []
        self.key = None

    @staticmethod
    def _valid(stamp_ns, xy):
        return (type(stamp_ns) is int and stamp_ns > 0 and len(xy) == 2
                and all(math.isfinite(float(v)) for v in xy))

    @staticmethod
    def _fit(rows):
        t = np.array([(s-rows[-1][0])/1e9 for s, _ in rows])
        xy = np.array([p for _, p in rows])
        a = np.column_stack([t, np.ones(len(t))])
        coeff = np.linalg.lstsq(a, xy, rcond=None)[0]
        error = float(np.max(np.linalg.norm(a @ coeff-xy, axis=1)))
        return coeff[0], error

    def observe(self, stamp_ns, xy):
        """Called only after the caller accepts a current measurement."""
        if not self._valid(stamp_ns, xy):
            self.clear()
            return
        if self.history and not 0 < stamp_ns-self.history[-1][0] <= round(self.max_gap_s*1e9):
            self.clear()
        self.history.append((stamp_ns, tuple(map(float, xy))))
        while self.history and stamp_ns-self.history[0][0] > round(self.window_s*1e9):
            self.history.popleft()
        self.model = None
        self.blocked = None
        self.last_check = None
        self.clear_confirmation()

    def freeze(self, reason):
        self.blocked = reason
        self.clear_confirmation()

    def _reference(self):
        rows = list(self.history)
        if len(rows) < 8 or (rows[-1][0]-rows[0][0])/1e9 < self.min_span_s:
            return None, 'insufficient_accepted_motion'
        velocity, error = self._fit(rows)
        speed = float(np.linalg.norm(velocity))
        if not self.min_speed_m_s <= speed <= self.max_speed_m_s:
            return None, 'stationary_or_excessive_speed_freeze'
        middle = len(rows)//2
        first, _ = self._fit(rows[:middle+1])
        last, _ = self._fit(rows[middle:])
        if error > self.fit_error_m or np.linalg.norm(first-last) > self.velocity_error_m_s:
            return None, 'unstable_accepted_motion'
        return dict(stamp_ns=rows[-1][0], xy=rows[-1][1], velocity=velocity,
                    speed=speed, fit_error_m=error, samples=len(rows)), None

    def check(self, stamp_ns, xy, *, eligible, key):
        evidence = dict(ready=False)

        def reject(reason, *, latch=False):
            if latch:
                self.freeze(reason)
            else:
                self.clear_confirmation()
            return False, dict(evidence, reason=reason)

        if not self._valid(stamp_ns, xy):
            return reject('invalid_motion_candidate', latch=True)
        if self.last_check is not None and stamp_ns <= self.last_check:
            return reject('motion_time_not_increasing', latch=True)
        self.last_check = stamp_ns
        if self.blocked:
            return reject(self.blocked)
        if not eligible:
            return reject('qualified_measurement_required')
        if self.model is None:
            self.model, reason = self._reference()
            if self.model is None:
                return reject(reason)
        model = self.model
        age = (stamp_ns-model['stamp_ns'])/1e9
        evidence.update(reference_stamp_ns=model['stamp_ns'], age_s=age,
                        velocity_m_s=model['velocity'].tolist(), speed_m_s=model['speed'],
                        fit_error_m=model['fit_error_m'], history_samples=model['samples'])
        if not 0 < age <= self.horizon_s:
            return reject('motion_horizon_expired', latch=True)
        # The uncertainty has a hard cap; time alone never admits a candidate.
        predicted = np.asarray(model['xy']) + model['velocity']*age
        innovation = float(np.linalg.norm(np.asarray(xy)-predicted))
        limit = min(.20, .10+.10*age)
        evidence.update(innovation_m=innovation, innovation_limit_m=limit)
        if innovation > limit:
            return reject('outside_frozen_motion_envelope', latch=True)
        if self.pending:
            dt = (stamp_ns-self.pending[-1][0])/1e9
            if dt > self.max_gap_s or key != self.key:
                self.clear_confirmation()
            elif math.dist(xy, self.pending[-1][1]) > .05+(model['speed']+.25)*dt:
                return reject('recovery_candidate_jump', latch=True)
        self.key = key
        self.pending.append((stamp_ns, tuple(map(float, xy))))
        elapsed = (stamp_ns-self.pending[0][0])/1e9
        evidence.update(elapsed_s=elapsed, samples=len(self.pending))
        if elapsed < self.confirm_s or len(self.pending) < self.min_samples:
            return False, dict(evidence, reason='confirming_motion')
        velocity, error = self._fit(self.pending)
        delta = float(np.linalg.norm(velocity-model['velocity']))
        along = float(np.dot(velocity, model['velocity'])/model['speed']**2)
        evidence.update(candidate_velocity_m_s=velocity.tolist(), velocity_delta_m_s=delta,
                        candidate_fit_error_m=error, along_track_ratio=along)
        if error > self.fit_error_m or delta > self.velocity_error_m_s or not .5 <= along <= 1.5:
            return reject('recovery_velocity_inconsistent', latch=True)
        return True, dict(evidence, ready=True, reason='bounded_motion_confirmed')
