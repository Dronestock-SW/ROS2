"""One-tag RAW/TDMA pairing before observation processing.

RAW has no boot/session fields. Its immediately following diagnostic supplies
that identity. A missing, late, duplicate or inconsistent pair is never an
observation. Host receipt times remain those of RAW, not of the diagnostic.
"""
from collections import Counter, deque
from dataclasses import dataclass

from drone_uwb.contracts.protocol import InvalidSample, finite, integer


def uint32(value):
    return integer(value) and 0 <= value <= 0xffffffff


def validate_sideband(msg):
    kind = msg.get('type')
    if kind == 'uwb_tdma_epoch':
        names = ('boot_id', 'session', 'sf_seq', 'seq', 'schedule_id',
                 'start_offset_us', 'end_offset_us', 'epoch_span_us', 'log_drops')
        if (not all(uint32(msg.get(k)) for k in names)
                or type(msg.get('deadline_valid')) is not bool
                or type(msg.get('local_overrun')) is not bool):
            raise InvalidSample('invalid_tdma_diagnostic')
    elif kind == 'jetson_pose_ack':
        if (any(type(msg.get(k)) is not bool for k in ('accepted', 'fix', 'lora_eligible'))
                or not isinstance(msg.get('reason'), str)
                or not (uint32(msg.get('pose_seq')) if msg['accepted'] else msg.get('pose_seq') is None)):
            raise InvalidSample('invalid_pose_ack')
    elif kind == 'lora_rx':
        if (not all(finite(msg.get(k)) for k in ('rssi_dbm', 'snr_db'))
                or not isinstance(msg.get('payload'), str)):
            raise InvalidSample('invalid_lora_rx')
    else:
        return False
    return True


@dataclass
class Dispatch:
    reason: str
    message: dict = None
    mono_ns: int = 0
    ros_ns: int = 0
    tdma: dict = None
    reset_history: bool = False
    clear_status: bool = False


class TdmaGate:
    def __init__(self, tag_id, mode='auto', max_pair_age_s=.05):
        if mode not in ('auto', 'required', 'disabled'):
            raise ValueError('invalid_tdma_mode')
        if not finite(max_pair_age_s) or not 0 < max_pair_age_s <= .15:
            raise ValueError('invalid_tdma_pair_age')
        self.tag_id, self.mode = tag_id, mode
        self.max_age_ns = int(max_pair_age_s*1e9)
        self.counts = Counter()
        self.disconnect()

    def disconnect(self):
        self.required = self.mode == 'required'
        self.pending = None
        self.identity = None
        self.retired = deque(maxlen=32)
        self.last_sf = None
        self.last_pair_ns = None
        self.last_boot_status_ns = None
        self.last_log_drops = None

    def discard_pending(self, reason='parse_error'):
        if self.pending is not None:
            self.counts[reason] += 1
            self.pending = None

    def ingest(self, msg, mono_ns, ros_ns):
        if (not isinstance(msg, dict) or not integer(msg.get('schema'))
                or msg['schema'] != 1 or msg.get('tag_id') != self.tag_id):
            self.discard_pending('schema_or_tag_mismatch')
            raise InvalidSample('schema_or_tag_mismatch')
        kind = msg.get('type')
        if kind == 'uwb_raw_status':
            if self.mode == 'auto' and str(msg.get('firmware', '')).startswith('uwb-tag-tdma40-'):
                self.required = True
            if msg.get('event') == 'boot':
                self.discard_pending('source_boot')
                self.last_boot_status_ns = mono_ns
            return Dispatch('status', msg, mono_ns, ros_ns)
        if kind == 'uwb_raw_cycle':
            if not self.required:
                return Dispatch('raw', msg, mono_ns, ros_ns)
            if self.tag_id not in ('5', '6'):
                raise InvalidSample('unsupported_tdma_tag')
            if (not uint32(msg.get('seq')) or not integer(msg.get('cycle_start_us'))
                    or not integer(msg.get('cycle_end_us'))
                    or not 0 <= msg['cycle_start_us'] <= msg['cycle_end_us']):
                self.discard_pending('invalid_cycle')
                raise InvalidSample('invalid_cycle')
            if self.pending is not None:
                self.counts['tdma_missing'] += 1
            self.pending = (msg, mono_ns, ros_ns)
            return Dispatch('awaiting_tdma')
        if not validate_sideband(msg):
            self.discard_pending('unsupported_type')
            raise InvalidSample('unsupported_type')
        if kind != 'uwb_tdma_epoch' or not self.required:
            # A new diagnostic on an unidentified legacy stream cannot prove the
            # preceding RAW. Auto mode becomes strict for subsequent cycles.
            if kind == 'uwb_tdma_epoch' and self.mode == 'auto':
                self.required = True
            return Dispatch('sideband_'+kind)
        pending, self.pending = self.pending, None
        if pending is None:
            raise InvalidSample('tdma_without_raw')
        raw, raw_mono, raw_ros = pending
        if msg['seq'] != raw['seq']:
            raise InvalidSample('tdma_seq_mismatch')
        if not 0 <= mono_ns-raw_mono <= self.max_age_ns:
            raise InvalidSample('tdma_pair_expired')
        if msg['schedule_id'] != 0x4001:
            raise InvalidSample('unsupported_tdma_schedule')
        identity = (msg['boot_id'], msg['session'])
        if identity in self.retired:
            raise InvalidSample('retired_tdma_session')
        changed = self.identity is not None and identity != self.identity
        boot_changed = changed and identity[0] != self.identity[0]
        if not changed and self.last_sf is not None:
            delta = (msg['sf_seq']-self.last_sf) & 0xffffffff
            if not 0 < delta < 0x80000000:
                raise InvalidSample('duplicate_or_old_tdma_epoch')
        fresh_boot = (self.last_boot_status_ns is not None and
                      (self.last_pair_ns is None or self.last_boot_status_ns > self.last_pair_ns))
        offset = 1000 if self.tag_id == '5' else 12500
        in_slot = offset <= msg['start_offset_us'] <= msg['end_offset_us'] < offset+10700
        expected_valid = raw.get('valid_mask') == 15 and in_slot
        if msg['deadline_valid'] != expected_valid:
            raise InvalidSample('tdma_deadline_conflict')
        reason = 'raw'
        if msg['local_overrun'] or not msg['deadline_valid']:
            reason = 'tdma_deadline_rejected'
        drops = None if changed else self.last_log_drops
        if drops is not None and msg['log_drops'] != drops:
            reason = 'tdma_log_drop'
        # Check the physical sample span as well as sequence identity. RAW is
        # still validated independently by the existing measurement validator.
        samples = raw.get('sample_time_us')
        if reason == 'raw' and (not isinstance(samples, list) or len(samples) != 4
                or not all(integer(x) for x in samples)
                or msg['epoch_span_us'] != max(samples)-min(samples)):
            raise InvalidSample('tdma_sample_span_mismatch')
        # Commit identity only after validating the pair. A malformed session
        # transition must not hide the reset needed by the next valid pair.
        if changed:
            self.retired.append(self.identity)
        self.identity, self.last_sf = identity, msg['sf_seq']
        self.last_pair_ns, self.last_log_drops = mono_ns, msg['log_drops']
        if reason != 'raw':
            return Dispatch(reason, reset_history=changed,
                            clear_status=boot_changed and not fresh_boot)
        return Dispatch('raw', raw, raw_mono, raw_ros, dict(msg), changed,
                        boot_changed and not fresh_boot)
