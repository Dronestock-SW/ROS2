import copy
import hashlib
from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'python'))
from sangwon_sensors.delivery import PendingObservations
from sangwon_web.common import encode


class Tests(unittest.TestCase):
    def setUp(self):
        self.packet = {'schema_version': 'sangwon-scan-observation/2', 'scope': 'SYNTHETIC_SENSOR_REPLAY',
            'flight_authority': False, 'observation_id': 'O', 'request_id': 'R',
            'context': {'drone_id': 'TEST-D', 'execution_id': 'E', 'task_id': 'S', 'window_id': 'W'},
            'producer_id': 'P', 'kind': 'QR_SCANNER', 'sequence': 1, 'observed_monotonic_s': 1.,
            'observed_at': '2026-10-04T00:00:00Z', 'data': {'raw': 'SYNTHETIC_PRIVATE_PAYLOAD'}}
        self.receipt = {k: copy.deepcopy(self.packet[k]) for k in ('observation_id', 'request_id', 'context', 'producer_id', 'kind', 'sequence')}
        self.receipt.update(schema_version='sangwon-scan-receipt/1', scope='SYNTHETIC_SENSOR_REPLAY', flight_authority=False,
            transport_sha256=hashlib.sha256(encode(self.packet)).hexdigest(), status='INGRESS_ACCEPTED', accepted=True,
            acceptance_scope='INPUT_VALIDATION_ONLY', source_observed_monotonic_s=1., source_observed_at=self.packet['observed_at'])
        self.queue = PendingObservations();self.queue.offer(self.packet, .25, 1.01)

    def test_lost_packet_and_receipt_retry_exact_bytes_without_retiming(self):
        expected = encode(self.packet);self.packet['data']['raw'] = 'changed'
        self.assertEqual(self.queue.due(1.01), [expected])
        self.assertEqual(self.queue.due(1.02), [])
        self.assertEqual(self.queue.due(1.05), [expected])
        self.assertEqual(self.queue.due(1.1), [expected])
        self.assertTrue(self.queue.receive(self.receipt, 1.11))
        self.assertEqual(self.queue.due(1.12), [])
        self.assertEqual(self.queue.accepted, 1)
        self.assertFalse(self.queue.receive(self.receipt, 1.13))

    def test_wrong_receipt_cannot_delete_pending_source_data(self):
        for key, value in (('transport_sha256', '0' * 64), ('request_id', 'other'), ('kind', 'QR_CAMERA'),
                ('sequence', 2), ('context', {'execution_id': 'other'}), ('flight_authority', True),
                ('source_observed_monotonic_s', 1.1), ('source_observed_at', 'other'),
                ('acceptance_scope', 'TASK_SUCCEEDED')):
            receipt = copy.deepcopy(self.receipt);receipt[key] = value
            self.assertFalse(self.queue.receive(receipt, 1.1), key)
            self.assertIn('O', self.queue.pending)

    def test_source_age_expires_without_reconnection_extension(self):
        self.queue.due(1.01);self.assertEqual(self.queue.due(1.251), [])
        self.assertFalse(self.queue.receive(self.receipt, 1.252));self.assertEqual(self.queue.expired, 1)
        with self.assertRaises(ValueError):self.queue.offer(self.packet, .25, 1.3)

    def test_retryable_unknown_delivery_and_definitive_rejection(self):
        receipt = copy.deepcopy(self.receipt);receipt.update(status='RETRYABLE', accepted=False)
        self.assertFalse(self.queue.receive(receipt, 1.1));self.assertIn('O', self.queue.pending)
        receipt['status'] = 'REJECTED'
        self.assertTrue(self.queue.receive(receipt, 1.11));self.assertEqual(self.queue.rejected, 1)

    def test_bounded_queue_id_conflict_and_no_high_rate_stream_buffer(self):
        queue = PendingObservations(capacity=1);queue.offer(self.packet, .25, 1.01)
        other = copy.deepcopy(self.packet);other['data']['raw'] = 'other'
        with self.assertRaisesRegex(ValueError, 'ID_CONFLICT'):queue.offer(other, .25, 1.02)
        other['observation_id'] = 'O2'
        with self.assertRaisesRegex(ValueError, 'BUFFER_FULL'):queue.offer(other, .25, 1.02)
        other['kind'] = 'MARKER'
        with self.assertRaises(ValueError):queue.offer(other, .25, 1.02)
        self.assertNotIn('SYNTHETIC_PRIVATE_PAYLOAD', str(self.receipt))


if __name__ == '__main__':unittest.main()
