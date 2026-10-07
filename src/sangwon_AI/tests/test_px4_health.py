import copy
from pathlib import Path
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))
from sangwon_sensors.px4 import host_checks, read_report


def report():
    fields = {'state': {'connected': True, 'armed': False, 'guided': False, 'manual_input': True, 'mode': 'POSCTL', 'system_status': 3},
              'extended_state': {'landed_state': 1, 'vtol_state': 0},
              'battery': {'present': True, 'voltage_v': 16.2, 'current_a': None, 'percentage': .8, 'health': 0},
              'rc': {'channel_count': 8, 'rssi': 255, 'rssi_known': False}}
    return {'schema_version': 'sangwon-px4-health/1', 'scope': 'MAVROS_TRANSPORT_ONLY', 'profile': 'HOST_OBSERVE',
            'observation_source': 'MAVROS_TOPICS_UNVERIFIED_AIRCRAFT', 'ros_domain_id': 1,
            'boot_id': 'test-boot', 'monitor_session_id': 'a' * 32, 'monitor_seq': 1,
            'generated_monotonic_s': 100, 'display_ttl_s': 5, 'can_start': False, 'flight_authority': False,
            'physical_output_enabled': False, 'source_sample_time_verified': False, 'aircraft_identity_verified': False,
            'runtime_code': 'OK', 'streams': {key: {'topic': '/mavros/' + key, 'publisher_count': 1,
                'received_count': 1, 'accepted_count': 1, 'rejected_count': 0, 'status': 'LIVE', 'code': 'OK',
                'transport_observation_valid': True, 'header_age_ms': 0, 'max_header_age_ms': 1000, 'fields': value}
                for key, value in fields.items()}}


class Px4HealthTests(unittest.TestCase):
    def test_px4_xyz_diagnostic_never_becomes_flight_readiness(self):
        r = report()
        xyz = copy.deepcopy(r['streams']['state'])
        xyz.update(topic='/mavros/local_position/pose', max_header_age_ms=200,
                   fields=dict(frame_id='map', x_m=1., y_m=2., z_m=.4, qw=1., qx=0., qy=0., qz=0.))
        r['streams']['local_position'] = xyz
        checks = host_checks(r, 'test-boot', 100.1)
        self.assertEqual(checks[-1]['status'], 'PASS')
        self.assertFalse(checks[-1]['required_for_flight'])
        self.assertIn('warehouse alignment/fusion unverified', checks[-1]['detail'])
        self.assertEqual(host_checks(r, 'test-boot', 100.3)[-1]['status'], 'WARN')
        xyz['fields']['frame_id'] = 'uwb_map'
        self.assertEqual(host_checks(r, 'test-boot', 100.1)[0]['status'], 'UNKNOWN')

    def test_only_optional_host_checks_and_rc_not_takeover(self):
        checks = host_checks(report(), 'test-boot', 100.1)
        self.assertEqual(len(checks), 5)
        self.assertTrue(all(c['id'].startswith('OBS_PX4_') and not c['required_for_flight'] for c in checks))
        self.assertEqual(checks[-1]['status'], 'WARN')
        self.assertIn('takeover/RC link semantics unverified', checks[-1]['detail'])

    def test_boot_report_source_authority_and_malformed_are_rejected(self):
        for field, value in (('boot_id', 'other'), ('observation_source', 'SYNTHETIC_MAVROS_TEST'),
                             ('ros_domain_id', 181), ('flight_authority', True), ('can_start', True),
                             ('physical_output_enabled', True), ('source_sample_time_verified', True),
                             ('aircraft_identity_verified', True), ('monitor_seq', True), ('monitor_session_id', 'bad'),
                             ('generated_monotonic_s', float('nan'))):
            r = report(); r[field] = value
            self.assertEqual(host_checks(r, 'test-boot', 100.1)[0]['status'], 'UNKNOWN', field)
        for now in (99.9, 105, float('inf')):
            self.assertEqual(host_checks(report(), 'test-boot', now)[0]['status'], 'UNKNOWN')

    def test_source_age_increases_even_when_file_is_fresh(self):
        checks = host_checks(report(), 'test-boot', 101.1)
        self.assertTrue(all(c['status'] == 'WARN' and 'EXPIRED' in c['detail'] for c in checks[1:]))

    def test_unavailable_battery_and_disconnected_state_are_visible(self):
        r = report(); r['streams']['battery']['fields']['percentage'] = None
        r['streams']['state']['fields']['connected'] = False
        checks = host_checks(r, 'test-boot', 100.1)
        self.assertEqual(checks[1]['status'], 'WARN'); self.assertEqual(checks[3]['status'], 'WARN')
        self.assertIn('percentage=unknown', checks[3]['detail'])

    def test_no_cached_values_on_stale_or_ambiguous_data(self):
        for key, value in (('publisher_count', 2), ('transport_observation_valid', False),
                           ('status', 'STALE'), ('header_age_ms', float('nan')), ('rejected_count', 1)):
            r = report(); r['streams']['state'][key] = value
            self.assertEqual(host_checks(r, 'test-boot', 100.1)[0]['status'], 'UNKNOWN', key)
        r = report(); r['streams']['battery']['fields']['percentage'] = 1.2
        self.assertEqual(host_checks(r, 'test-boot', 100.1)[0]['status'], 'UNKNOWN')
        r = report(); r['streams']['rc']['fields']['channels'] = [1500] * 8
        self.assertEqual(host_checks(r, 'test-boot', 100.1)[0]['status'], 'UNKNOWN')

    def test_missing_and_oversized_private_reports(self):
        with tempfile.TemporaryDirectory() as d:
            path = Path(d) / 'health.json'
            self.assertEqual(read_report(path), {})
            path.write_text('{"large":"' + 'x' * 65536 + '"}')
            self.assertEqual(read_report(path), {})


if __name__ == '__main__':
    unittest.main()
