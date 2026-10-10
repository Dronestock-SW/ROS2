"""Bounded, asynchronous evidence storage. No ROS or vehicle control imports."""
from collections import Counter
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import shutil
import time

from drone_uwb.integration.async_recording import AsyncRecording
from drone_uwb.integration.async_checkpoint import AsyncCheckpoint, atomic_text


def json_value(value):
    """Keep nonfinite sensor values distinguishable from missing/null values."""
    if isinstance(value, float) and not math.isfinite(value):
        return 'NaN' if math.isnan(value) else ('Infinity' if value > 0 else '-Infinity')
    if isinstance(value, dict):
        return {str(k): json_value(v) for k, v in value.items()}
    if isinstance(value, (tuple, list)):
        return [json_value(v) for v in value]
    return value


def atomic_json(path, value):
    path = Path(path)
    temporary = path.with_suffix('.tmp')
    temporary.write_text(json.dumps(json_value(value), ensure_ascii=False, allow_nan=False,
                                    indent=2)+'\n', encoding='utf-8')
    temporary.replace(path)


def clock_record():
    return dict(monotonic_ns=time.monotonic_ns(), wall_ns=time.time_ns(),
                monotonic_raw_ns=monotonic_raw_ns(),
                utc=datetime.now(timezone.utc).isoformat())


def monotonic_raw_ns():
    """Linux unslewed clock, diagnostic only; never a replacement ROS stamp."""
    clock = getattr(time, 'CLOCK_MONOTONIC_RAW', None)
    return time.clock_gettime_ns(clock) if clock is not None else None


def process_identity():
    path = Path('/proc/self/stat')
    # comm may contain spaces or parentheses. Field 22 follows the last ')'.
    ticks = path.read_text().rsplit(')', 1)[1].split()[19] if path.exists() else None
    return dict(pid=os.getpid(), start_ticks=ticks)


def boot_id():
    path = Path('/proc/sys/kernel/random/boot_id')
    return path.read_text(encoding='utf-8').strip() if path.exists() else None


class GroundObservation:
    """Conservative retention evidence, not aircraft readiness."""
    def __init__(self, started_ns):
        self.started_ns = started_ns
        self.proof = dict(all_ground=True, all_disarmed=True, all_connected=True, all_fresh=True,
                          state_count=0, landed_count=0, max_state_gap_s=0., max_landed_gap_s=0.)
        self.last = {}

    def observe(self, topic, data, mono_ns, ros_ns, header_ns):
        kind = {'/mavros/state':'state', '/mavros/extended_state':'landed'}.get(topic)
        if kind is None:
            return
        p = self.proof
        p['all_fresh'] &= header_ns is not None and header_ns > 0 and 0 <= ros_ns-header_ns <= 1_500_000_000
        p[kind+'_count'] += 1
        if kind in self.last:
            gap = (mono_ns-self.last[kind])/1e9
            p['max_'+kind+'_gap_s'] = max(p['max_'+kind+'_gap_s'], gap)
            p['all_fresh'] &= gap >= 0
        else:
            p['first_'+kind+'_delay_s'] = (mono_ns-self.started_ns)/1e9
            p['all_fresh'] &= mono_ns >= self.started_ns
        self.last[kind] = mono_ns
        if kind == 'state':
            p['all_disarmed'] &= data.get('armed') is False
            p['all_connected'] &= data.get('connected') is True
        else:
            p['all_ground'] &= data.get('landed_state') == 1

    def report(self, now_ns):
        return dict(self.proof, **{'last_'+k+'_age_s':(now_ns-v)/1e9 for k,v in self.last.items()})


class Capture:
    def __init__(self, directory, metadata, *, max_bytes=192*1024*1024,
                 min_free_bytes=512*1024*1024, writer_factory=AsyncRecording):
        if max_bytes <= 0 or min_free_bytes < 0:
            raise ValueError('invalid_capture_capacity')
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        if shutil.disk_usage(self.directory).free < min_free_bytes:
            raise OSError('capture_disk_reserve_low')
        self.started = clock_record()
        self.ground_observation = GroundObservation(self.started['monotonic_ns'])
        self.max_bytes, self.queued_bytes = max_bytes, 0
        self.required = list(metadata.get('required_topics', []))
        self.expected = list(metadata.get('topics', {}))
        self.stats, self.message_ids, self.fc_state = {}, Counter(), None
        self.transitions = []
        self.error = None
        self.rejected = 0
        self.closed = False
        self.manifest = dict(metadata, schema=1, started=self.started, boot_id=boot_id(),
            process=process_identity(),
            scope='manual_flight_receive_only', flight_authorized=False,
            alignment_confirmed=False, timing_confirmed=False,
            fc_commands_sent=0, parameter_writes=0,
            nonfinite_encoding='NaN/Infinity/-Infinity strings; null remains missing',
            max_event_bytes=max_bytes, min_free_bytes=min_free_bytes)
        atomic_json(self.directory/'manifest.json', self.manifest)
        stream = (self.directory/'events.jsonl').open('x', encoding='utf-8', buffering=64*1024)
        self.writer = writer_factory({'events': stream}, min_free_bytes=min_free_bytes)
        self.checkpoints = None
        self.stop_requested = False
        # Initial I/O precedes subscription setup. Subsequent checkpoints are async.
        atomic_json(self.directory/'summary.json', self.report())
        self.checkpoints = AsyncCheckpoint(self.directory/'summary.json', write=self._write_checkpoint)

    def _write_checkpoint(self, path, text):
        # Even metadata queries can block on storage. Poll STOP on the status worker.
        self.stop_requested |= (self.directory/'STOP').exists()
        atomic_text(path, text)

    @property
    def storage_error(self):
        return self.error or self.writer.error or (self.checkpoints.error if self.checkpoints else None)

    def add(self, topic, type_name, data, *, mono_ns, ros_ns, header_ns=None):
        if self.closed or self.storage_error:
            self.rejected += 1
            return False
        stat = self.stats.setdefault(topic, dict(received=0, queued=0, max_gap_s=0.,
            gaps_over_250ms=0, negative_header_age_count=0, max_header_age_s=None,
            first_mono_ns=mono_ns, last_mono_ns=mono_ns))
        if stat['received']:
            gap = (mono_ns-stat['last_mono_ns'])/1e9
            stat['max_gap_s'] = max(stat['max_gap_s'], gap)
            stat['gaps_over_250ms'] += int(gap > .25)
        stat['received'] += 1
        stat['last_mono_ns'] = mono_ns
        if header_ns is not None and header_ns > 0:
            age = (ros_ns-header_ns)/1e9
            stat['negative_header_age_count'] += int(age < 0)
            stat['max_header_age_s'] = max(age, stat['max_header_age_s']) if stat['max_header_age_s'] is not None else age
        row = dict(topic=topic, type=type_name, received_monotonic_ns=mono_ns,
                   received_ros_ns=ros_ns, header_ns=header_ns, data=data)
        line = json.dumps(json_value(row), ensure_ascii=False, allow_nan=False,
                          separators=(',', ':'))+'\n'
        size = len(line.encode('utf-8'))
        if self.queued_bytes+size > self.max_bytes:
            self.error = 'capture_byte_limit'
            self.rejected += 1
            return False
        if not self.writer.write('events', line):
            self.error = self.writer.error or 'capture_enqueue_failed'
            self.rejected += 1
            return False
        self.queued_bytes += size
        stat['queued'] += 1
        self.ground_observation.observe(topic, data, mono_ns, ros_ns, header_ns)
        if type_name == 'mavros_msgs/msg/Mavlink':
            self.message_ids[f'{topic}:{data.get("msgid")}'] += 1
        if topic == '/mavros/state':
            state = {k: data.get(k) for k in ('connected', 'armed', 'mode')}
            if state != self.fc_state:
                if len(self.transitions) < 256:
                    self.transitions.append(dict(monotonic_ns=mono_ns, **state))
                self.fc_state = state
        return True

    def report(self, reason='recording', *, completed=False):
        error = self.storage_error
        flow = any(self.message_ids.get('/uas1/mavlink_source:'+str(i), 0) for i in (100, 106))
        flow = flow or any(self.stats.get(t, {}).get('queued', 0) for t in self.expected if '/px4flow/' in t)
        now = time.monotonic_ns()
        rate = self.queued_bytes/max((now-self.started['monotonic_ns'])/1e9, 1e-9)
        remaining = max(0, self.max_bytes-self.queued_bytes)
        return dict(schema=1, scope='manual_flight_receive_only', updated=clock_record(),
            duration_s=(now-self.started['monotonic_ns'])/1e9, stopped=completed,
            average_event_bytes_per_s=rate, remaining_event_bytes=remaining,
            estimated_capacity_remaining_s=remaining/rate if rate else None,
            reason=reason, error=error, queued_event_bytes=self.queued_bytes,
            rejected_events=self.rejected, writer_drained=completed and error is None,
            required_topics_missing=[t for t in self.required if not self.stats.get(t, {}).get('queued')],
            optional_topics_missing=[t for t in self.expected if t not in self.required and not self.stats.get(t, {}).get('queued')],
            topics={t:dict(s, last_receipt_age_s=(now-s['last_mono_ns'])/1e9) for t,s in self.stats.items()},
            mavlink_message_ids=dict(self.message_ids), fc_state=self.fc_state,
            ground_observation=self.ground_observation.report(now),
            state_transitions=self.transitions, raw_optical_flow_received=bool(flow),
            px4_ulog_attached=False, independent_reference_verified=False,
            alignment_confirmed=False, timing_confirmed=False, flight_authorized=False,
            fc_commands_sent=0, parameter_writes=0,
            limitation='Capture completeness is not flight readiness or calibration proof. Join matching PX4 ULog for internal fusion state.')

    def checkpoint(self):
        if self.closed:
            return
        self.writer.flush()
        # Serialize here so later callback mutations cannot alter the queued snapshot.
        text = json.dumps(json_value(self.report()), ensure_ascii=False, allow_nan=False, indent=2)+'\n'
        if not self.checkpoints.submit(text):
            self.error = self.storage_error or 'checkpoint_enqueue_failed'

    def close(self, reason):
        self.closed = True
        try:
            self.writer.close()
        except OSError as exc:
            self.error = self.error or str(exc)
        summary = self.report(reason, completed=True)
        try:
            self.checkpoints.close(json.dumps(json_value(summary), ensure_ascii=False,
                                              allow_nan=False, indent=2)+'\n')
        except OSError as exc:
            self.error = self.error or str(exc)
            summary = self.report(reason, completed=True)
        return summary


def inspect_capture(directory):
    """A persisted summary is not proof that this boot's recorder is alive."""
    root = Path(directory)
    manifest = json.loads((root/'manifest.json').read_text(encoding='utf-8'))
    summary = json.loads((root/'summary.json').read_text(encoding='utf-8'))
    same_boot = manifest['boot_id'] is not None and manifest['boot_id'] == boot_id()
    age = (time.monotonic_ns()-summary['updated']['monotonic_ns'])/1e9 if same_boot else None
    fresh = age is not None and 0 <= age <= 5
    return dict(summary, inspected=clock_record(), same_host_boot=same_boot,
                checkpoint_age_s=age, recording_active=same_boot and fresh
                and not summary['stopped'] and not summary['error'])


def add_marker(directory, label):
    directory = Path(directory)
    manifest = json.loads((directory/'manifest.json').read_text(encoding='utf-8'))
    if not label.strip() or len(label) > 160:
        raise ValueError('marker_requires_1_to_160_characters')
    if manifest['boot_id'] != boot_id():
        raise ValueError('marker_must_use_capture_host_boot')
    row = dict(clock_record(), label=label, kind='operator_annotation_not_measurement',
               boot_id=manifest['boot_id'])
    with (directory/'markers.jsonl').open('a', encoding='utf-8') as stream:
        stream.write(json.dumps(row, ensure_ascii=False, allow_nan=False)+'\n')
    return row


def config_evidence(paths):
    evidence = []
    for path in paths:
        path = Path(path)
        content = path.read_bytes()
        if len(content) > 1024*1024:
            raise ValueError('config_evidence_exceeds_1MiB')
        evidence.append(dict(name=path.name, sha256=hashlib.sha256(content).hexdigest(),
                             content_utf8=content.decode('utf-8')))
    return evidence
