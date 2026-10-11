"""Check B4 against measured height without changing its XY or inventing motion.

The H80 fit has a free common squared-range term. A small fit residual therefore
does not imply consistency with the measured antenna height. Compare each range
at its own acquisition time, using the fitted horizontal velocity. This is a
consistency veto, not independent position truth or a slow-motion detector.
"""
import math

import numpy as np


class B4HeightIntegrity:
    def __init__(self, anchors, *, limit_m, window_s):
        self.anchors = np.asarray(anchors, float)
        self.limit_m = limit_m
        self.window_s = window_s
        self.blocked = False
        self.clear_confirmation()

    def clear_confirmation(self):
        self.first_good_us = None
        self.last_good_us = None
        self.good_count = 0

    def check(self, result):
        stamp = result['time_us']
        evidence = dict(state='unavailable', checked=False, blocked=self.blocked,
                        limit_m=self.limit_m, reason='measured_height_unavailable')
        heights = result.get('sample_height_m', [result.get('height_m')]*4)
        measured = (result.get('height_source') in ('measured_tof_imu', 'gazebo_tof_imu')
                    and len(heights) == 4
                    and all(type(h) in (int, float) and math.isfinite(h) for h in heights))
        if not measured:
            # A missing/ground-model height cannot clear an observed conflict.
            if self.last_good_us is not None and not 0 < stamp-self.last_good_us <= 100_000:
                self.clear_confirmation()
            return not self.blocked, evidence

        b = result['models']['B']
        xy = np.asarray(b.get('xy_m'), float)
        velocity = np.asarray(b.get('velocity_m_s'), float)
        times = np.asarray(result.get('sample_time_us', [stamp]*4))
        ranges = np.asarray(result['cal_slant_m'], float)
        if (xy.shape != (2,) or velocity.shape != (2,) or times.shape != (4,)
                or ranges.shape != (4,) or not np.issubdtype(times.dtype, np.integer)
                or not all(np.isfinite(v).all() for v in (xy, velocity, ranges))
                or np.any(ranges <= 0) or np.any(times > stamp)
                or np.any(stamp-times > round(self.window_s*1e6))):
            self.blocked = True
            self.clear_confirmation()
            return False, dict(evidence, state='invalid', blocked=True,
                               reason='invalid_b4_physical_inputs')

        # Never freeze at an old XY or compare sequential anchors as simultaneous.
        dt = (times-stamp)/1e6
        positions = np.column_stack([xy[None, :] + dt[:, None]*velocity, heights])
        residual = np.linalg.norm(positions-self.anchors, axis=1)-ranges
        rms = float(np.sqrt(np.mean(residual**2)))
        evidence.update(checked=True, physical_range_rms_m=rms,
                        predicted_minus_observed_m=residual.tolist(),
                        sample_time_us=times.tolist(), motion_compensated=True)
        if rms >= self.limit_m:
            self.blocked = True
            self.clear_confirmation()
            return False, dict(evidence, state='inconsistent', blocked=True,
                               reason='b4_height_inconsistent')

        if self.blocked:
            if self.last_good_us is None or not 0 < stamp-self.last_good_us <= 100_000:
                self.clear_confirmation()
                self.first_good_us = stamp
            self.last_good_us = stamp
            self.good_count += 1
            elapsed = (stamp-self.first_good_us)/1e6
            evidence.update(recovery_samples=self.good_count, recovery_elapsed_s=elapsed)
            if self.good_count < 4 or elapsed < .15:
                return False, dict(evidence, state='recovering', blocked=True,
                                   reason='b4_height_recovery_pending')
            self.blocked = False
            self.clear_confirmation()
        return True, dict(evidence, state='consistent', blocked=False,
                          reason='b4_height_consistent')
