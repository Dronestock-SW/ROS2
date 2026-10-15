"""Single Tag B schema 2: strict RAW, four-timestamp clock and pose provenance.

This module performs no I/O and never substitutes receive time for event time.
The only clock conversion occurs in SingleTagLink.envelope().
"""
from collections import Counter, deque
import math
import secrets

import numpy as np

from .validation import Cycle, InvalidInput
from drone_uwb.contracts.protocol import finite


DOMAIN = 'esp32_monotonic_boot_us'


def uint(value, maximum=0xffffffff):
    return type(value) is int and 0 <= value <= maximum


def identity(message):
    if (message.get('schema') != 2 or type(message.get('schema')) is not int
            or message.get('tag_id') != '6'
            or not uint(message.get('boot_id')) or not message['boot_id']
            or not uint(message.get('session')) or not message['session']):
        raise InvalidInput('single_tag_identity')
    return message['boot_id'], message['session']


def cycle_fields(message, mono, ros):
    identity(message)
    if message.get('clock_domain') != DOMAIN or any(k in message for k in ('sf_seq', 'schedule_id')):
        raise InvalidInput('single_tag_clock_or_legacy_fields')
    seq, start, end, mask = (message.get(k) for k in ('seq', 'cycle_start_us', 'cycle_end_us', 'valid_mask'))
    if (not uint(seq) or not uint(mask, 15) or not uint(start, 2**53-1)
            or not uint(end, 2**53-1) or not 0 <= end-start <= 25000
            or message.get('period_us') not in (20000, 25000)):
        raise InvalidInput('single_tag_cycle')
    arrays = [message.get(k) for k in ('raw_slant_m', 'sample_time_us', 'failure', 'attempts', 'rf_map_span_us')]
    if any(type(a) is not list or len(a) != 4 for a in arrays):
        raise InvalidInput('single_tag_arrays')
    values, times, failures, attempts, spans = arrays
    indices = []
    for i in range(4):
        if not uint(failures[i], 13) or not uint(attempts[i], 2) or not uint(spans[i], 65535):
            raise InvalidInput('single_tag_diagnostics')
        if mask & (1 << i):
            if (failures[i] != 0 or attempts[i] < 1 or not finite(values[i])
                    or not 0 < values[i] <= 80 or not uint(times[i], 2**53-1)
                    or not start <= times[i] <= end or spans[i] > 500):
                raise InvalidInput('single_tag_measurement')
            indices.append(i)
        elif failures[i] == 0 or values[i] is not None or times[i] is not None:
            raise InvalidInput('single_tag_missing_must_be_null')
    if (type(message.get('uwb_ready')) is not bool or type(message.get('clock_sync_valid')) is not bool
            or type(message.get('deadline_missed')) is not bool or not uint(message.get('clock_generation'))):
        raise InvalidInput('single_tag_flags')
    return Cycle(seq, start, end, mask, np.array([v if v is not None else np.nan for v in values]),
                 list(times), list(failures), indices, host_mono_ns=mono, ros_ns=ros,
                 extra={k: message[k] for k in ('boot_id', 'session', 'period_us')})


class FourTimestampClock:
    """Bounded weighted fit of NTP midpoints; USB asymmetry remains uncertainty."""
    def __init__(self):
        self.reset()

    def reset(self):
        self.points = deque(maxlen=120)
        self.generation = secrets.randbelow(0xffffffff)+1
        self.last_us = None
        self.anchor_tag = self.anchor_host = 0.
        self.alpha = 1.
        self.uncertainty_us = math.inf

    def ready(self, now):
        return (len(self.points) >= 4 and self.last_us is not None
                and 0 <= now-self.last_us <= 2_000_000 and self.uncertainty_us <= 5000)

    def add(self, t1, t2, t3, t4):
        if not all(uint(t, 2**53-1) for t in (t1, t2, t3, t4)) or t3 < t2 or t4 <= t1:
            raise InvalidInput('sync_timestamp_order')
        rtt = (t4-t1)-(t3-t2)
        if not 0 <= rtt <= 12000 or t4-t1 > 100000:
            raise InvalidInput('sync_rtt')
        if self.last_us is not None and t4-self.last_us > 2_000_000:
            self.reset()
        point = ((t2+t3)/2., (t1+t4)/2., rtt)
        if self.points and (point[0] <= self.points[-1][0] or point[1] <= self.points[-1][1]):
            raise InvalidInput('sync_reordered')
        self.points.append(point)
        while self.points and t4-self.points[0][1] > 30_000_000:
            self.points.popleft()
        # Centering avoids cancellation with a long-running host monotonic clock.
        self.anchor_tag = point[0]
        xs = np.array([p[0]-self.anchor_tag for p in self.points])
        ys = np.array([p[1]-point[1] for p in self.points])
        weights = np.array([1./max(500., p[2])**2 for p in self.points])
        xmean, ymean = np.average(xs, weights=weights), np.average(ys, weights=weights)
        alpha = 1.
        if len(self.points) >= 16 and xs[-1]-xs[0] >= 5_000_000:
            alpha = float(np.sum(weights*(xs-xmean)*(ys-ymean))/np.sum(weights*(xs-xmean)**2))
            if not .999 <= alpha <= 1.001:
                self.reset()
                raise InvalidInput('sync_drift')
        self.alpha = alpha
        # Each exchange bounds the offset without assuming symmetric USB delay.
        # Intersect the intervals; a slow exchange cannot widen a tighter one.
        centers = [p[1]-alpha*(p[0]-self.anchor_tag) for p in self.points]
        lower = max(center-p[2]/2 for center,p in zip(centers,self.points))
        upper = min(center+p[2]/2 for center,p in zip(centers,self.points))
        if upper < lower:
            self.reset()
            raise InvalidInput('sync_inconsistent_intervals')
        self.anchor_host = (lower+upper)/2
        self.uncertainty_us = (upper-lower)/2+500.  # RF bracket; USB asymmetry is bounded above.
        self.last_us = t4

    def host_us(self, tag):
        return self.anchor_host+self.alpha*(tag-self.anchor_tag)

    def host_ns(self, tag):
        return round(self.host_us(tag)*1000)

    def diagnostics(self, now):
        return dict(method='four_timestamp_affine', ready=self.ready(now), generation=self.generation,
                    alpha=self.alpha, samples=len(self.points),
                    uncertainty_us=self.uncertainty_us if math.isfinite(self.uncertainty_us) else None,
                    last_exchange_host_us=self.last_us, fixed_delay_calibrated=False)


class SingleTagLink:
    def __init__(self):
        self.counts = Counter()
        self.retired = set()
        self.disconnect()

    def disconnect(self):
        self.source = None
        self.status_at = None
        self.host_session = secrets.randbelow(0xffffffff)+1
        self.clock = FourTimestampClock()
        self.pending = None
        self.next_sync = 0
        self.sync_seq = self.pose_seq = 0
        self.last_seq = self.last_end = None
        self.recent = deque(maxlen=80)
        self.last_pose_source = None

    def command(self, kind, **values):
        return dict(type=kind, schema=2, tag_id='6', boot_id=self.source[0], session=self.source[1],
                    host_session=self.host_session, **values)

    def request(self, now_us):
        if self.source is None or self.status_at is None or now_us-self.status_at > 2_500_000:
            return None
        if self.pending is not None and now_us-self.pending[1] < 100000:
            return None
        if now_us < self.next_sync:
            return None
        self.sync_seq = (self.sync_seq+1) & 0xffffffff
        self.pending = self.sync_seq, now_us
        self.next_sync = now_us+250000
        return self.command('time_sync', seq=self.sync_seq, t1_host_us=now_us)

    def ingest(self, message, mono_ns, ros_ns):
        """Return a receiver envelope and an optional protocol commit command."""
        now = mono_ns//1000
        kind = message.get('type')
        command = None
        if kind == 'uwb_raw_status':
            source = identity(message)
            if (message.get('rf_protocol') != 2 or message.get('clock_domain') != DOMAIN
                    or message.get('timestamp_method') != 'rf_poll_final_midpoint'
                    or message.get('period_us') not in (20000, 25000)
                    or message.get('range_bias_applied') is not False
                    or message.get('temporal_filter_applied') is not False):
                raise InvalidInput('single_tag_status_contract')
            if source != self.source:
                if source in self.retired:
                    raise InvalidInput('single_tag_retired_session')
                if self.source:
                    self.retired.add(self.source)
                self.disconnect()
                self.source = source
            self.status_at = now
        elif kind == 'time_sync_reply':
            if (identity(message) != self.source or message.get('host_session') != self.host_session
                    or self.pending is None or message.get('seq') != self.pending[0]
                    or message.get('t1_host_us') != self.pending[1]):
                raise InvalidInput('sync_reply_identity')
            seq, t1 = self.pending
            self.pending = None
            self.clock.add(t1, message.get('t2_tag_us'), message.get('t3_tag_us'), now)
            if self.clock.ready(now):
                command = self.command('time_sync_commit', seq=seq, generation=self.clock.generation,
                                       uncertainty_us=math.ceil(self.clock.uncertainty_us))
        elif kind == 'uwb_raw_cycle':
            cycle = cycle_fields(message, mono_ns, ros_ns)
            if identity(message) != self.source or self.status_at is None or not 0 <= now-self.status_at <= 2_500_000:
                raise InvalidInput('single_tag_status_unavailable')
            if self.last_seq is not None:
                delta = (cycle.seq-self.last_seq) & 0xffffffff
                if not 0 < delta < 0x80000000 or cycle.end_us <= self.last_end:
                    raise InvalidInput('single_tag_duplicate_or_reordered')
            self.last_seq, self.last_end = cycle.seq, cycle.end_us
        elif message.get('schema') != 2:
            raise InvalidInput('single_tag_legacy_rejected')
        event = dict(host_received_monotonic_ns=mono_ns, host_received_ros_ns=ros_ns, message=message)
        meta = dict(host_session=self.host_session, clock=self.clock.diagnostics(now), mapped=False)
        if (kind == 'uwb_raw_cycle' and self.clock.ready(now)
                and message['clock_sync_valid'] and message['clock_generation'] == self.clock.generation):
            samples = [self.clock.host_ns(t) if t is not None else None for t in cycle.sample_us]
            end = self.clock.host_ns(cycle.end_us)
            # A point estimate slightly in the future is rejected, never clamped to receipt.
            if not 0 <= mono_ns-end <= 150_000_000 or any(t is not None and t > mono_ns for t in samples):
                raise InvalidInput('single_tag_mapped_age')
            meta.update(mapped=True, cycle_end_host_monotonic_ns=end, sample_host_monotonic_ns=samples)
            self.recent.append((cycle.seq, tuple(cycle.sample_us), mono_ns, self.clock.generation))
        event['single_tag'] = meta
        return event, command

    def pose_command(self, result, now_ns):
        """Only a published B_TF result tied to a retained RAW can produce a pose."""
        if (result.get('published') is not True or result.get('ok') is not True
                or self.source is None or result.get('source_identity') != list(self.source)
                or result.get('host_session') != self.host_session or not self.clock.ready(now_ns//1000)):
            raise InvalidInput('pose_unqualified')
        seq, sample = result.get('seq'), result.get('time_us')
        row = next((r for r in self.recent if r[0] == seq and sample in r[1]), None)
        xy = result.get('xy_m')
        if (row is None or row[3] != self.clock.generation or not 0 <= now_ns-row[2] <= 400_000_000
                or not 0 <= now_ns-self.clock.host_ns(sample) <= 500_000_000
                or self.last_pose_source is not None and sample <= self.last_pose_source
                or not isinstance(xy, list) or len(xy) != 2 or not all(finite(v) and abs(v) <= 10000 for v in xy)):
            raise InvalidInput('pose_source_stale_or_duplicate')
        self.last_pose_source = sample
        self.pose_seq = (self.pose_seq+1) & 0xffffffff
        return self.command('jetson_pose', seq=self.pose_seq, source_seq=seq, sample_time_us=sample,
                            clock_domain=DOMAIN, fix=True, x_m=xy[0], y_m=xy[1],
                            quality=25*sum(t is not None for t in row[1]), reason='ok')


class MappedSingleTagInput:
    """B_TF validation of already mapped observations; does not fit a clock."""
    def __init__(self):
        self.key = None
        self.retired = set()
        self.seq = self.end = None

    def accept(self, event):
        msg, meta = event['message'], event['single_tag']
        source = identity(msg)
        key = (*source, meta['host_session'], meta['clock']['generation'])
        changed = key != self.key
        if changed:
            if key in self.retired:
                raise InvalidInput('single_tag_retired_mapping')
            if self.key:
                self.retired.add(self.key)
            self.key, self.seq, self.end = key, None, None
        if msg['type'] != 'uwb_raw_cycle':
            return None, changed
        cycle = cycle_fields(msg, event['host_received_monotonic_ns'], event['host_received_ros_ns'])
        if self.seq is not None and (not 0 < (cycle.seq-self.seq) & 0xffffffff < 0x80000000 or cycle.end_us <= self.end):
            raise InvalidInput('single_tag_duplicate_or_reordered')
        cycle.seq_gap = ((cycle.seq-self.seq) & 0xffffffff)-1 if self.seq is not None else 0
        self.seq, self.end = cycle.seq, cycle.end_us
        if (meta.get('mapped') is not True or meta['clock'].get('ready') is not True
                or msg['clock_sync_valid'] is not True or msg['clock_generation'] != key[3]
                or meta['clock'].get('method') != 'four_timestamp_affine'):
            raise InvalidInput('single_tag_clock_unsynced')
        samples = meta['sample_host_monotonic_ns']
        end = meta['cycle_end_host_monotonic_ns']
        if (not uint(end, 2**63-1) or not 0 <= cycle.host_mono_ns-end <= 150_000_000
                or type(samples) is not list or len(samples) != 4):
            raise InvalidInput('single_tag_mapped_age')
        for i in range(4):
            if i in cycle.indices:
                if not uint(samples[i], 2**63-1) or not 0 <= end-samples[i] <= 25_000_000:
                    raise InvalidInput('single_tag_mapped_sample')
            elif samples[i] is not None:
                raise InvalidInput('single_tag_missing_mapped_time')
        return cycle, changed
