import copy
import json
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'python'))
from sangwon_sensors.perception import PerceptionHealth, config, host_checks


def configuration():
    return json.loads((ROOT / 'config/perception.observe.json').read_text())


class PerceptionTests(unittest.TestCase):
    def make(self):
        health = PerceptionHealth(configuration(), 'boot')
        for stream in health.streams.values():
            stream.publisher_count = 1
        return health

    def test_source_staleness_is_not_refreshed_by_monitor_heartbeat(self):
        h = self.make()
        h.streams['image'].stamped(100, 'camera', 100.01, 1, {'width': 640, 'height': 480})
        self.assertEqual(h.snapshot(1.01, 't')['streams']['image']['status'], 'LIVE')
        for t in (2, 3, 4):
            row = h.snapshot(t, 'new-heartbeat')['streams']['image']
            self.assertEqual(row['status'], 'STALE')
            self.assertFalse(row['source_observation_valid'])

    def test_duplicate_and_older_source_stamp_do_not_create_new_observations(self):
        h = self.make();s = h.streams['markers']
        s.stamped(100, 'camera', 100.01, 1, {'marker_count': 0})
        s.stamped(100, 'camera', 100.02, 1.01, {'marker_count': 0})
        self.assertEqual(s.accepted_count, 1)
        self.assertEqual(s.received_mono, 1)
        self.assertEqual(s.view(1.02)['code'], 'SOURCE_TIMESTAMP_NOT_NEW')
        s.stamped(99, 'camera', 100.02, 1.01, {})
        self.assertEqual(s.accepted_count, 1)

    def test_future_stale_nan_and_missing_frame_are_rejected(self):
        for stamp, frame in ((100.001, 'camera'), (99, 'camera'), (float('nan'), 'camera'), (100, ''), (0, 'camera')):
            h = self.make();s = h.streams['image'];s.stamped(stamp, frame, 100, 1, {})
            self.assertFalse(s.view(1)['source_observation_valid'])
            self.assertEqual(s.accepted_count, 0)

    def test_invalid_latest_frame_masks_prior_live_evidence(self):
        h = self.make();s = h.streams['image']
        s.stamped(100, 'camera', 100.01, 1, {})
        s.stamped(100.02, 'camera', 100.03, 1.02, {}, valid=False)
        self.assertEqual(s.view(1.03)['status'], 'WARN')
        self.assertFalse(s.view(1.03)['source_observation_valid'])

    def test_legacy_qr_cannot_become_scan_or_source_observation(self):
        h = self.make();s = h.streams['camera_qr'];s.unstamped_qr(20, 1)
        report = h.snapshot(1, 't');row = report['streams']['camera_qr']
        self.assertFalse(row['source_observation_valid']);self.assertIsNone(row['source_age_ms'])
        self.assertEqual(row['status'], 'WARN');self.assertEqual(row['accepted_count'], 0)
        self.assertNotIn('raw', row);self.assertNotIn('data', row)
        self.assertFalse(report['can_start']);self.assertFalse(report['flight_authority'])

    def test_image_calibration_reference_and_resolution_must_match(self):
        for frame, width in (('other_camera', 640), ('camera', 320)):
            h = self.make()
            h.streams['image'].stamped(100, 'camera', 100, 1, {'width': 640, 'height': 480})
            h.streams['camera_info'].stamped(100, frame, 100, 1, {'width': width, 'height': 480})
            self.assertEqual(h.snapshot(1, 't')['streams']['camera_info']['code'], 'IMAGE_CALIBRATION_FRAME_OR_SIZE_MISMATCH')

    def test_ambiguous_publishers_cannot_appear_live(self):
        for count in (None, 0, 2):
            h = self.make();s = h.streams['image'];s.publisher_count = count
            s.stamped(100, 'camera', 100, 1, {})
            self.assertFalse(s.view(1)['source_observation_valid'])

    def test_host_display_recomputes_sensor_age_and_checks_boot_and_authority(self):
        h = self.make();h.streams['image'].stamped(100, 'camera', 100, 1, {'width': 640, 'height': 480})
        report = h.snapshot(1, 't')
        good = {c['id']: c for c in host_checks(report, 'boot', 1.01)}
        self.assertEqual(good['OBS_IMAGE']['status'], 'PASS')
        self.assertFalse(good['OBS_IMAGE']['required_for_flight'])
        expired = {c['id']: c for c in host_checks(report, 'boot', 1.6)}
        self.assertEqual(expired['OBS_IMAGE']['status'], 'WARN')
        for changes in ({'boot_id': 'old-boot'}, {'flight_authority': True}, {'generated_monotonic_s': float('nan')}, {'runtime_code': 'invalid text'}):
            bad = copy.deepcopy(report);bad.update(changes)
            self.assertEqual(host_checks(bad, 'boot', 1.01)[0]['status'], 'UNKNOWN')
        self.assertEqual(host_checks(report, 'boot', 6)[0]['status'], 'UNKNOWN')

    def test_configuration_is_bounded_and_explicit(self):
        for field, value in (('flight_authority', True), ('ros_domain_id', True), ('ros_domain_id', 233)):
            cfg = configuration();cfg[field] = value
            with self.assertRaises(ValueError):config(cfg)

    def test_synthetic_ros_report_is_not_live_host_evidence(self):
        cfg = configuration();cfg['observation_source'] = 'SYNTHETIC_ROS_TEST'
        h = PerceptionHealth(cfg, 'boot');h.streams['image'].publisher_count = 1
        h.streams['image'].stamped(100, 'camera', 100, 1, {'width': 640, 'height': 480})
        self.assertEqual(host_checks(h.snapshot(1, 't'), 'boot', 1)[0]['status'], 'UNKNOWN')
        for topic in ('camera/image', '/camera//image', '/camera/image/', '/camera/12bad'):
            cfg = configuration();cfg['topics']['image'] = topic
            with self.assertRaises(ValueError):config(cfg)


if __name__ == '__main__':
    unittest.main()
