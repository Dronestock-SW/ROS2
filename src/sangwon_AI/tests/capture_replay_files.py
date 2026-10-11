"""File contract tests; synthetic data only, never ROS or vehicle access."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest

BINARY = Path(sys.argv.pop(1)).resolve()


class ReplayFiles(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.manifest = dict(schema=1, scope='manual_flight_receive_only', tag='B',
                             tag_id='6', ros_domain_id=2, boot_id='fixture', source_revision='test')
        self.row = dict(topic='/mavros/state', type='mavros_msgs/msg/State',
            received_monotonic_ns=10_000_000_000, received_ros_ns=1791544480000000000,
            header_ns=1791544480000000000, data=dict(header=dict(frame_id='',
                stamp=dict(sec=1791544480, nanosec=0)), connected=True, armed=False, mode='POSCTL'))

    def run_capture(self, events=None):
        (self.root/'manifest.json').write_text(json.dumps(self.manifest), encoding='utf-8')
        (self.root/'events.jsonl').write_bytes(events if events is not None else
            (json.dumps(self.row)+'\n').encode())
        return subprocess.run([str(BINARY), '--capture', str(self.root)], capture_output=True,
                              text=True, timeout=10)

    def test_digest_and_no_runtime_authority(self):
        result = self.run_capture()
        self.assertEqual(result.returncode, 0, result.stderr)
        report = json.loads(result.stdout)
        self.assertTrue(report['streams']['state']['sample_valid'])
        self.assertEqual(report['input']['events_sha256'], hashlib.sha256((self.root/'events.jsonl').read_bytes()).hexdigest())
        self.assertEqual(report['streams']['uwb_xy']['reason'], 'MISSING')
        self.assertFalse(report['can_start'])
        self.assertFalse(report['flight_authority'])
        self.assertFalse(report['physical_output_enabled'])

    def test_truncated_and_malformed_have_no_success_report(self):
        for data in (b'', b'{', json.dumps(self.row).encode(), b'{}\n', b'{"a":1,"a":2}\n'):
            with self.subTest(data=data[:30]):
                result = self.run_capture(data)
                self.assertEqual(result.returncode, 2)
                self.assertEqual(result.stdout, '')

    def test_tag_domain_mismatch(self):
        self.manifest['ros_domain_id'] = 1
        self.assertEqual(self.run_capture().returncode, 2)

    def test_receipt_order_does_not_sort_away_reset(self):
        older = dict(self.row, received_monotonic_ns=self.row['received_monotonic_ns']-1)
        result = self.run_capture(('\n'.join(map(json.dumps, [self.row, older]))+'\n').encode())
        self.assertEqual(result.returncode, 2)
        self.assertEqual(result.stdout, '')

    def test_bad_message_remains_visible_as_rejection(self):
        self.row['data']['connected'] = 'true'
        report = json.loads(self.run_capture().stdout)
        self.assertFalse(report['streams']['state']['sample_valid'])
        self.assertEqual(report['streams']['state']['rejections']['BOOLEAN_REQUIRED'], 1)


if __name__ == '__main__':
    unittest.main()
