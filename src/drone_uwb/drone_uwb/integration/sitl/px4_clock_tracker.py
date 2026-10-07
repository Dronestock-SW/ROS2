"""Track actual PX4 boot-clock replies without extrapolating simulation time."""
import time

from drone_uwb.integration.sitl.sitl_clock_probe import timesync_sample


class PX4ClockTracker:
    def __init__(self, mav, *, host_now_ns=time.monotonic_ns):
        self.mav, self.host_now_ns = mav, host_now_ns
        self.pending = {}
        self.last_request_ns = self.last_progress_host_ns = None
        self.px4_ns = None
        self.last_reply_sent_ns = None
        self.fault = None

    def poll(self):
        now = self.host_now_ns()
        if self.fault:
            return None
        if self.last_request_ns is not None and now < self.last_request_ns:
            self.fault = 'host_time_reversed'
            return None
        self.pending = {stamp: value for stamp, value in self.pending.items()
                        if now-stamp <= 200_000_000}
        if self.last_request_ns is None or now-self.last_request_ns >= 50_000_000:
            self.mav.timesync_send(0, now)
            self.pending[now] = True
            self.last_request_ns = now
            return dict(type='px4_clock_request', sent_host_ns=now)
        return None

    def handle(self, message):
        if message.get_type() != 'TIMESYNC' or getattr(message, 'ts1', None) not in self.pending:
            return None
        sent = message.ts1
        now = self.host_now_ns()
        try:
            record = timesync_sample(message, sent_ns=sent, received_ns=now)
        except (ValueError, TypeError, AttributeError):
            return None
        self.pending.pop(sent)
        if self.last_reply_sent_ns is not None and sent < self.last_reply_sent_ns:
            return dict(type='px4_clock_reply_rejected', reason='older_request_reply', **record)
        if record['round_trip_ns'] > 50_000_000:
            return dict(type='px4_clock_reply_rejected', reason='round_trip_exceeded', **record)
        self.last_reply_sent_ns = sent
        if self.px4_ns is not None and record['px4_boot_ns'] < self.px4_ns:
            self.fault = 'px4_time_reversed'
        elif record['px4_boot_ns'] != self.px4_ns:
            self.px4_ns = record['px4_boot_ns']
            self.last_progress_host_ns = now
        return dict(type='px4_clock_reply', fault=self.fault, **record)

    def now_us(self):
        now = self.host_now_ns()
        if self.fault:
            raise ValueError(self.fault)
        if self.px4_ns is None:
            raise ValueError('px4_clock_unavailable')
        if not 0 <= now-self.last_progress_host_ns <= 200_000_000:
            raise ValueError('px4_clock_stalled')
        return self.px4_ns//1000

    def sample_age_s(self, stamp_us, receipt_host_ns):
        if type(stamp_us) is not int or stamp_us <= 0 or type(receipt_host_ns) is not int:
            raise ValueError('invalid_px4_sample_clock')
        sample_age = (self.now_us()-stamp_us)/1e6
        residence = (self.host_now_ns()-receipt_host_ns)/1e9
        if sample_age < 0 or residence < 0:
            # A telemetry packet may be newer than the last clock reply.
            # The next request can establish its age; do not extrapolate now.
            raise ValueError('px4_sample_newer_than_clock_or_host_reversed')
        return max(sample_age, residence)
