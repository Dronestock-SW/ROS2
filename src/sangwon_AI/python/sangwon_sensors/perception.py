"""Bound sensor evidence without storing images, marker poses or QR contents."""
import math
import re
import uuid

SCHEMA = 'sangwon-perception-health/1'
REPORT_TTL_S = 5
CHANNELS = ('image', 'camera_info', 'markers', 'scanner_qr', 'camera_qr')


def config(value):
    if not isinstance(value, dict) or value.get('flight_authority') is not False:
        raise ValueError('PERCEPTION_AUTHORITY_REFUSED')
    if value.get('observation_source') not in ('ROS_TOPICS_UNVERIFIED_AIRCRAFT', 'SYNTHETIC_ROS_TEST'):
        raise ValueError('PERCEPTION_SOURCE_REQUIRED')
    domain = value.get('ros_domain_id')
    if type(domain) is not int or not 0 <= domain <= 232:
        raise ValueError('INVALID_ROS_DOMAIN')
    topics = value.get('topics')
    limits = value.get('max_source_age_ms')
    if not isinstance(topics, dict) or set(topics) != set(CHANNELS):
        raise ValueError('INVALID_PERCEPTION_TOPICS')
    if not isinstance(limits, dict) or set(limits) != {'image', 'camera_info', 'markers'}:
        raise ValueError('INVALID_PERCEPTION_LIMITS')
    for topic in topics.values():
        if not isinstance(topic, str) or len(topic) > 192 or not re.fullmatch(r'(/[A-Za-z_][A-Za-z0-9_]*)+', topic):
            raise ValueError('INVALID_PERCEPTION_TOPIC')
    if len(set(topics.values())) != len(topics):
        raise ValueError('AMBIGUOUS_PERCEPTION_TOPICS')
    for age in limits.values():
        if type(age) is not int or not 10 <= age <= 10000:
            raise ValueError('INVALID_PERCEPTION_LIMITS')
    return value


class Stream:
    def __init__(self, topic, max_age_ms=None):
        self.topic, self.max_age_ms = topic, max_age_ms
        self.received_count = self.accepted_count = self.rejected_count = 0
        self.publisher_count = None
        self.last_stamp = self.received_mono = self.source_age_s = None
        self.frame_id = None
        self.code = 'NO_OBSERVATION'
        self.metadata = {}

    def reject(self, code):
        self.rejected_count += 1
        self.code = code

    def stamped(self, stamp, frame_id, utc_now, mono_now, metadata, valid=True):
        self.received_count += 1
        if not valid:
            self.reject('INVALID_SENSOR_MESSAGE')
            return
        if (type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp <= 0
                or not isinstance(frame_id, str) or not 1 <= len(frame_id) <= 128
                or not math.isfinite(utc_now) or not math.isfinite(mono_now)):
            self.reject('SOURCE_TIMESTAMP_OR_FRAME_INVALID')
            return
        age = utc_now - stamp
        if age < 0 or age * 1000 >= self.max_age_ms:
            self.reject('SOURCE_TIMESTAMP_NOT_CURRENT')
            return
        if self.last_stamp is not None and stamp <= self.last_stamp:
            self.reject('SOURCE_TIMESTAMP_NOT_NEW')
            return
        self.last_stamp, self.received_mono, self.source_age_s = stamp, mono_now, age
        self.frame_id, self.metadata = frame_id, metadata
        self.accepted_count += 1
        self.code = 'OK'

    def unstamped_qr(self, length, mono_now):
        # A receive timestamp is not the decoding/image timestamp. There is no
        # execution/task/window binding on legacy std_msgs/String either.
        self.received_count += 1
        self.received_mono = mono_now
        if type(length) is not int or not 0 < length <= 65536:
            self.reject('INVALID_LEGACY_QR_MESSAGE')
        else:
            self.code = 'LEGACY_QR_WITHOUT_SOURCE_TIME_OR_SCAN_CONTEXT'

    def view(self, mono_now):
        result = {'topic': self.topic, 'publisher_count': self.publisher_count,
            'received_count': self.received_count, 'accepted_count': self.accepted_count,
            'rejected_count': self.rejected_count, 'source_observation_valid': False,
            'source_age_ms': None, 'max_source_age_ms': self.max_age_ms,
            'frame_id': self.frame_id, 'code': self.code, 'status': 'UNKNOWN'}
        if self.max_age_ms is None:
            if self.received_mono is not None:
                result['status'] = 'WARN'
            return result
        if self.received_mono is None:
            if self.rejected_count:
                result['status'] = 'WARN'
            return result
        age = self.source_age_s + mono_now - self.received_mono
        if not math.isfinite(age) or age < 0 or age * 1000 >= self.max_age_ms:
            result.update(status='STALE', code='SOURCE_EXPIRED')
            return result
        result['source_age_ms'] = round(age * 1000, 3)
        if self.code != 'OK':
            result['status'] = 'WARN'
            return result
        if self.publisher_count != 1:
            result.update(status='WARN', code='TOPIC_PUBLISHER_NOT_UNIQUE')
            return result
        result.update(status='LIVE', source_observation_valid=True, **self.metadata)
        return result


class PerceptionHealth:
    def __init__(self, configuration, boot):
        self.config = config(configuration)
        self.boot, self.session, self.sequence = boot, str(uuid.uuid4()), 0
        self.streams = {name: Stream(topic, self.config['max_source_age_ms'].get(name))
                        for name, topic in self.config['topics'].items()}

    def snapshot(self, mono_now, generated_at, runtime_code='OK'):
        self.sequence += 1
        streams = {name: stream.view(mono_now) for name, stream in self.streams.items()}
        image, calibration = streams['image'], streams['camera_info']
        if image['status'] == calibration['status'] == 'LIVE':
            if (image['frame_id'] != calibration['frame_id'] or image['width'] != calibration['width']
                    or image['height'] != calibration['height']):
                calibration.update(status='WARN', code='IMAGE_CALIBRATION_FRAME_OR_SIZE_MISMATCH', source_observation_valid=False)
        return {'schema_version': SCHEMA, 'scope': 'PERCEPTION_TRANSPORT_ONLY',
            'boot_id': self.boot, 'monitor_session_id': self.session, 'monitor_seq': self.sequence,
            'generated_at': generated_at, 'generated_monotonic_s': mono_now, 'display_ttl_s': REPORT_TTL_S,
            'ros_domain_id': self.config['ros_domain_id'], 'runtime_code': runtime_code,
            'observation_source': self.config['observation_source'],
            'flight_authority': False, 'can_start': False, 'streams': streams,
            'remaining': ['AIRCRAFT_TOPIC_BINDING_NOT_VERIFIED', 'SCAN_CALIBRATION_NOT_APPROVED',
                          'LEGACY_QR_SOURCE_TIME_AND_CONTEXT_REQUIRED']}


def host_checks(report, boot, mono_now):
    """Produce host transport observations only, never BP/QS approval evidence."""
    unavailable = [{'id': 'OBS_PERCEPTION_MONITOR', 'status': 'UNKNOWN', 'required_for_flight': False,
        'detail': 'No current perception transport report',
        'operator_action': 'Check observer service, ROS domain and configured sensor topics'}]
    try:
        if (report['schema_version'] != SCHEMA or report['scope'] != 'PERCEPTION_TRANSPORT_ONLY'
                or report['boot_id'] != boot or report['can_start'] is not False or report['flight_authority'] is not False
                or report['observation_source'] != 'ROS_TOPICS_UNVERIFIED_AIRCRAFT'
                or report['display_ttl_s'] != REPORT_TTL_S
                or type(report['generated_monotonic_s']) not in (int, float)):
            return unavailable
        age = mono_now - report['generated_monotonic_s']
        if not math.isfinite(age) or not 0 <= age < REPORT_TTL_S:
            return unavailable
        streams = report['streams']
        if not isinstance(streams, dict) or set(streams) != set(CHANNELS):
            return unavailable
        code, domain = report['runtime_code'], report['ros_domain_id']
        if not isinstance(code, str) or not re.fullmatch(r'[A-Z0-9_]{1,96}', code) or type(domain) is not int or not 0 <= domain <= 232:
            return unavailable
        checks = [{'id': 'OBS_PERCEPTION_MONITOR', 'status': 'PASS' if code == 'OK' else 'UNKNOWN', 'required_for_flight': False,
            'detail': f'Read-only ROS subscriber; domain={domain}; {code}',
            'operator_action': 'Match configured ROS domain/topics to the aircraft; no flight authority is provided'}]
        for name in CHANNELS:
            s = streams[name]
            status = s['status']
            if status not in {'LIVE', 'STALE', 'WARN', 'UNKNOWN'}:
                return unavailable
            code = s['code']
            if not isinstance(code, str) or not re.fullmatch(r'[A-Z0-9_]{1,96}', code):
                return unavailable
            topic = s['topic']
            if not isinstance(topic, str) or len(topic) > 192 or not re.fullmatch(r'(/[A-Za-z_][A-Za-z0-9_]*)+', topic):
                return unavailable
            count = s['received_count']
            if type(count) is not int or not 0 <= count <= 2**63-1:
                return unavailable
            if status == 'LIVE':
                source_age, limit = s['source_age_ms'], s['max_source_age_ms']
                if (name not in ('image', 'camera_info', 'markers') or s['source_observation_valid'] is not True
                        or type(source_age) not in (int, float) or not math.isfinite(source_age) or source_age < 0
                        or type(limit) is not int or not 10 <= limit <= 10000):
                    return unavailable
                if source_age + age * 1000 >= limit:
                    status, code = 'STALE', 'SOURCE_EXPIRED'
            checks.append({'id': 'OBS_' + name.upper(), 'status': 'PASS' if status == 'LIVE' else ('WARN' if status in ('WARN','STALE') else 'UNKNOWN'),
                'required_for_flight': False, 'detail': f'{topic}; observations={count}; {code}',
                'operator_action': 'Confirm aircraft binding and approved calibration; transport observations do not grant flight readiness'})
        return checks
    except (KeyError, TypeError, ValueError, OverflowError):
        return unavailable
