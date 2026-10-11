import copy
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from sangwon_sensors.scan_window import CaptureGate


class Tests(unittest.TestCase):
    def setUp(self):
        self.request = {'schema_version': 'sangwon-scan-request/2', 'scope': 'SYNTHETIC_SENSOR_REPLAY',
            'flight_authority': False, 'physical_output_enabled': False, 'producer_id': 'source', 'active': True,
            'context': {'drone_id': 'TEST-D', 'execution_id': 'E', 'task_id': 'S', 'window_id': 'W'},
            'request_id': 'R', 'issued_monotonic_s': 1., 'expires_monotonic_s': 1.2,
            'observe_marker': True, 'open_reader': True, 'enable_camera': False}
        self.gate = CaptureGate('source');self.assertTrue(self.gate.update(self.request, 1.01))

    def test_source_capture_context_is_immutable(self):
        data = {'raw': 'SYNTHETIC'}
        packet = self.gate.capture('QR_SCANNER', data, 1.02, 1.03)
        data['raw'] = 'changed';self.request['context']['window_id'] = 'other'
        self.assertEqual(packet['data']['raw'], 'SYNTHETIC')
        self.assertEqual(packet['context']['window_id'], 'W')

    def test_capture_before_window_after_expiry_and_repeated_frame_rejected(self):
        for stamp, now in ((.99, 1.03), (1.05, 1.03), (float('nan'), 1.03), (1.02, 1.21)):
            with self.assertRaises(ValueError):self.gate.capture('QR_SCANNER', {}, stamp, now)
        self.gate.capture('MARKER', {}, 1.02, 1.03)
        with self.assertRaisesRegex(ValueError, 'CAPTURE_REPEATED'):self.gate.capture('MARKER', {}, 1.02, 1.04)

    def test_camera_cannot_capture_in_scanner_window(self):
        for kind in ('QR_CAMERA', 'CAMERA_ACK', 'LEGACY_QR'):
            with self.assertRaises(ValueError):self.gate.capture(kind, {}, 1.02, 1.03)

    def test_pause_scope_identity_and_expired_updates_close_window(self):
        for key, value in (('active', False), ('flight_authority', True), ('producer_id', 'other'),
                ('scope', 'FLIGHT'), ('expires_monotonic_s', 1.)):
            request = copy.deepcopy(self.request);request[key] = value
            self.assertFalse(self.gate.update(request, 1.01))
            with self.assertRaises(ValueError):self.gate.capture('QR_SCANNER', {}, 1.02, 1.03)

    def test_new_window_cannot_relabel_an_old_source_capture(self):
        packet = self.gate.capture('QR_SCANNER', {}, 1.02, 1.03)
        request = copy.deepcopy(self.request);request.update(request_id='R2', issued_monotonic_s=1.04)
        request['context']['window_id'] = 'W2';self.gate.update(request, 1.05)
        with self.assertRaises(ValueError):self.gate.capture('QR_SCANNER', {}, 1.02, 1.06)
        self.assertEqual(packet['context']['window_id'], 'W')


if __name__ == '__main__':unittest.main()
