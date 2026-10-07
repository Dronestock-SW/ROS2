"""Capture at the source inside a C++ issued window; never relabel legacy QR text."""
import copy
import datetime as dt
import math
import time
import uuid


class CaptureGate:
    def __init__(self, producer):
        if not isinstance(producer, str) or not 0 < len(producer) <= 128:
            raise ValueError('SCAN_PRODUCER_REQUIRED')
        self.producer = producer
        self.request = None
        self.sequence = 0
        self.last_capture = {}

    def update(self, request, now=None):
        now = time.monotonic() if now is None else now
        try:
            valid = (request['schema_version'] == 'sangwon-scan-request/2'
                and request['scope'] == 'SYNTHETIC_SENSOR_REPLAY'
                and request['flight_authority'] is False and request['physical_output_enabled'] is False
                and request['active'] is True and request['producer_id'] == self.producer
                and request['context']['drone_id'].startswith('TEST-')
                and isinstance(request['request_id'], str) and bool(request['request_id'])
                and math.isfinite(request['issued_monotonic_s']) and math.isfinite(request['expires_monotonic_s'])
                and request['issued_monotonic_s'] <= now < request['expires_monotonic_s']
                and 0 < request['expires_monotonic_s'] - now <= .201)
        except (KeyError, TypeError, AttributeError):
            valid = False
        self.request = copy.deepcopy(request) if valid else None
        return valid

    def capture(self, kind, data, observed_monotonic_s, now=None):
        now = time.monotonic() if now is None else now
        request = self.request
        if request is None or not math.isfinite(now) or now >= request['expires_monotonic_s']:
            raise ValueError('SCAN_REQUEST_EXPIRED')
        if (type(observed_monotonic_s) not in (float, int) or not math.isfinite(observed_monotonic_s)
                or not request['issued_monotonic_s'] <= observed_monotonic_s <= now):
            raise ValueError('SCAN_SOURCE_TIME_INVALID')
        if observed_monotonic_s <= self.last_capture.get(kind, -math.inf):
            raise ValueError('SCAN_CAPTURE_REPEATED')
        allowed = {'STATUS': request['active'], 'MARKER': request['observe_marker'],
            'CAMERA_ACK': request['enable_camera'], 'QR_CAMERA': request['enable_camera'],
            'QR_SCANNER': request['open_reader']}
        if not allowed.get(kind, False):
            raise ValueError('SCAN_SOURCE_NOT_REQUESTED')
        self.sequence += 1
        self.last_capture[kind] = observed_monotonic_s
        return {'schema_version': 'sangwon-scan-observation/2', 'scope': 'SYNTHETIC_SENSOR_REPLAY',
            'observation_id': str(uuid.uuid4()),
            'flight_authority': False, 'request_id': request['request_id'],
            'context': copy.deepcopy(request['context']), 'producer_id': self.producer,
            'sequence': self.sequence, 'observed_monotonic_s': observed_monotonic_s,
            'observed_at': dt.datetime.fromtimestamp(time.time() - (time.monotonic() - observed_monotonic_s), dt.timezone.utc).isoformat(),
            'kind': kind, 'data': copy.deepcopy(data)}
