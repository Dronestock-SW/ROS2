"""A received Gazebo clock for SITL, without wall-time extrapolation."""
import math
from threading import Lock
import time


class ClockUnavailable(ValueError):
    def __init__(self, reason, *, reset_required=False):
        super().__init__(reason)
        self.reset_required = reset_required


def simulation_clock_ns(message):
    seconds, nanoseconds = message.sim.sec, message.sim.nsec
    if (type(seconds) is not int or seconds < 0 or type(nanoseconds) is not int
            or not 0 <= nanoseconds < 1_000_000_000):
        raise ValueError('invalid_gazebo_clock')
    stamp = seconds*1_000_000_000+nanoseconds
    if stamp >= 2**63:
        raise ValueError('invalid_gazebo_clock')
    return stamp


class GazeboSimulationClock:
    """Latch simulation ticks; use host time only to detect a stalled stream.

    A future live adapter must feed this from the same world's /clock topic
    as PX4, and use the same clock domain for observation measurement stamps.
    """

    clock_domain = 'gazebo_sim_us'

    def __init__(self, *, max_stall_s=.25, host_now_ns=time.monotonic_ns):
        if (type(max_stall_s) not in (int, float) or not math.isfinite(max_stall_s)
                or max_stall_s <= 0):
            raise ValueError('positive_stall_timeout_required')
        self.max_stall_ns = round(max_stall_s*1e9)
        self.host_now_ns = host_now_ns
        self.lock = Lock()
        self.epoch = 0
        self._sim_ns = None
        self._progress_host_ns = None
        self._last_receipt_ns = None
        self._fault = None

    def update(self, sim_ns, *, received_host_ns=None):
        host_ns = self.host_now_ns() if received_host_ns is None else received_host_ns
        if (type(sim_ns) is not int or not 0 <= sim_ns < 2**63
                or type(host_ns) is not int or host_ns <= 0):
            raise ValueError('invalid_clock_sample')
        with self.lock:
            if self._fault:
                raise ClockUnavailable(self._fault, reset_required=True)
            if self._last_receipt_ns is not None and host_ns < self._last_receipt_ns:
                self._fault = 'host_clock_reversed'
            elif self._sim_ns is not None and sim_ns < self._sim_ns:
                self._fault = 'simulation_time_reversed'
            if self._fault:
                raise ClockUnavailable(self._fault, reset_required=True)
            self._last_receipt_ns = host_ns
            if self._sim_ns is None or sim_ns > self._sim_ns:
                self._sim_ns = sim_ns
                self._progress_host_ns = host_ns
                return True
            # Repeated ticks while paused must not renew freshness.
            return False

    def now_ns(self):
        host_ns = self.host_now_ns()
        with self.lock:
            if self._fault:
                raise ClockUnavailable(self._fault, reset_required=True)
            if self._sim_ns is None or self._sim_ns <= 0:
                raise ClockUnavailable('simulation_clock_unavailable')
            age_ns = host_ns-self._progress_host_ns
            if age_ns < 0:
                self._fault = 'host_clock_reversed'
                raise ClockUnavailable(self._fault, reset_required=True)
            if age_ns > self.max_stall_ns:
                raise ClockUnavailable('simulation_clock_stalled')
            return self._sim_ns

    def reset_epoch(self):
        """Caller must also clear observation history and PX4 sync evidence."""
        with self.lock:
            self.epoch += 1
            self._sim_ns = self._progress_host_ns = self._last_receipt_ns = None
            self._fault = None
