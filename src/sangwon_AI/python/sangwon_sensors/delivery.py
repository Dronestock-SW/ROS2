"""Memory-only source delivery; retry original bytes without refreshing capture age."""
import copy
import hashlib
import math
import time

from sangwon_web.common import encode


class PendingObservations:
    def __init__(self, capacity=32, retry_s=.025):
        if type(capacity) is not int or not 1 <= capacity <= 32 or not .01 <= retry_s <= .2:
            raise ValueError('INVALID_SCAN_DELIVERY_LIMITS')
        self.capacity, self.retry_s = capacity, retry_s
        self.pending = {}
        self.accepted = self.rejected = self.expired = 0

    def expire(self, now):
        for key in list(self.pending):
            if now >= self.pending[key]['deadline']:
                del self.pending[key];self.expired += 1

    def offer(self, packet, max_age_s, now=None):
        now = time.monotonic() if now is None else now
        if not math.isfinite(now) or type(max_age_s) not in (float, int) or not 0 < max_age_s <= 1:
            raise ValueError('INVALID_SCAN_DELIVERY_LIMITS')
        self.expire(now)
        if (packet['schema_version'] != 'sangwon-scan-observation/2'
                or packet['scope'] != 'SYNTHETIC_SENSOR_REPLAY' or packet['flight_authority'] is not False
                or packet['kind'] not in ('QR_SCANNER', 'QR_CAMERA', 'CAMERA_ACK')
                or not isinstance(packet['observation_id'], str) or not 0 < len(packet['observation_id']) <= 128
                or type(packet['observed_monotonic_s']) not in (float, int)
                or not math.isfinite(packet['observed_monotonic_s'])
                or not packet['observed_monotonic_s'] <= now < packet['observed_monotonic_s'] + max_age_s):
            raise ValueError('INVALID_SCAN_DELIVERY_PACKET')
        raw = encode(packet)
        if len(raw) > 128 * 1024:raise ValueError('SCAN_INPUT_TOO_LARGE')
        key = packet['observation_id']
        if key in self.pending:
            if self.pending[key]['raw'] != raw:raise ValueError('SCAN_OBSERVATION_ID_CONFLICT')
            return
        if len(self.pending) >= self.capacity:raise ValueError('SCAN_DELIVERY_BUFFER_FULL')
        self.pending[key] = {'raw': raw, 'packet': copy.deepcopy(packet), 'sha256': hashlib.sha256(raw).hexdigest(),
            'deadline': packet['observed_monotonic_s'] + max_age_s, 'next': now, 'attempts': 0}

    def due(self, now=None):
        now = time.monotonic() if now is None else now
        self.expire(now);rows = []
        for item in self.pending.values():
            if now >= item['next']:
                rows.append(item['raw']);item['next'] = now + self.retry_s;item['attempts'] += 1
        return rows

    def receive(self, receipt, now=None):
        now = time.monotonic() if now is None else now
        self.expire(now)
        if not isinstance(receipt, dict):return False
        key = receipt.get('observation_id');item = self.pending.get(key) if isinstance(key, str) else None
        if item is None:return False
        packet = item['packet']
        if (receipt.get('schema_version') != 'sangwon-scan-receipt/1'
                or receipt.get('scope') != 'SYNTHETIC_SENSOR_REPLAY' or receipt.get('flight_authority') is not False
                or receipt.get('transport_sha256') != item['sha256']
                or any(receipt.get(k) != packet[k] for k in ('request_id', 'context', 'producer_id', 'kind', 'sequence'))):
            return False
        if receipt.get('status') == 'RETRYABLE':return False
        if receipt.get('accepted') is True:
            if (receipt.get('acceptance_scope') != 'INPUT_VALIDATION_ONLY'
                    or receipt.get('source_observed_monotonic_s') != packet['observed_monotonic_s']
                    or receipt.get('source_observed_at') != packet['observed_at']):return False
            self.accepted += 1
        elif receipt.get('status') == 'REJECTED' and receipt.get('accepted') is False:
            self.rejected += 1
        else:return False
        del self.pending[key];return True
