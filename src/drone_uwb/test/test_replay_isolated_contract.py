"""Offline checks of preserved clocks and input provenance, without ROS."""
import copy
import hashlib
import importlib.util
import json
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

_tools = Path(__file__).parents[1]/'tools'
sys.path.insert(0,str(_tools))
_spec = importlib.util.spec_from_file_location('replay_btf_isolated',_tools/'replay_btf_isolated.py')
replay = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(replay)


class ReplayContractTest(unittest.TestCase):
    def test_unreadable_thermal_attribute_is_explicit_not_zero(self):
        replay.READ_ERRORS.clear()
        with patch.object(Path,'read_text',side_effect=TypeError('unreadable sysfs value')):
            self.assertIsNone(replay.read_text('/sys/mock/temp'))
        self.assertIn('/sys/mock/temp',replay.READ_ERRORS)

    def test_translation_preserves_sensor_and_monotonic_times(self):
        event = dict(host_received_ros_ns=111,host_received_monotonic_ns=99,
            message=dict(type='uwb_raw_cycle',seq=7,cycle_end_us=123,sample_time_us=[100,101,102,103]))
        original = copy.deepcopy(event)
        message = SimpleNamespace(data=json.dumps(event))
        replay.shift_message(message,event,10000)
        actual = json.loads(message.data)
        self.assertEqual(actual['host_received_ros_ns']-original['host_received_ros_ns'],10000)
        self.assertEqual(actual['host_received_monotonic_ns'],original['host_received_monotonic_ns'])
        self.assertEqual(actual['message'],original['message'])

    def test_translation_preserves_header_difference_and_zero_stamp(self):
        def message(value):
            sec,nanosec = divmod(value,10**9)
            return SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=sec,nanosec=nanosec)))
        a,b,zero = message(900_000_000),message(920_000_000),message(0)
        for m in (a,b,zero):
            replay.shift_message(m,None,10_200_000_000)
        def ns(m):
            return m.header.stamp.sec*10**9+m.header.stamp.nanosec
        self.assertEqual(ns(b)-ns(a),20_000_000)
        self.assertEqual(ns(a),11_100_000_000)
        self.assertEqual(ns(zero),0)

    def test_identity_does_not_replace_raw_source_age(self):
        row = dict(ordinal=42,topic='/uwb/received',received_ros_ns=300,
            received_monotonic_ns=299,data=dict(data=json.dumps(dict(host_received_ros_ns=100,
                host_received_monotonic_ns=99,message=dict(type='uwb_raw_cycle',seq=7,cycle_end_us=123)))))
        identity = replay.identity(row)
        self.assertEqual(identity['ordinal'],42)
        self.assertEqual(identity['source_received_ros_ns']-identity['source_host_received_ros_ns'],200)
        self.assertEqual(identity['cycle_end_us'],123)

    def fixture(self, root, *, reversed_time=False, external=False):
        config = dict(external_output_allowed=external,imu_topic='/imu',tof_topic='/tof')
        text = json.dumps(config)
        manifest = dict(config_evidence=[dict(name='field-btf.json',content_utf8=text,
                                              sha256=hashlib.sha256(text.encode()).hexdigest())])
        (root/'manifest.json').write_text(json.dumps(manifest),encoding='utf-8')
        topics = ['/uwb/received','/imu','/tof','/mavros/state','/mavros/extended_state',
                  '/mavros/timesync_status','/mavros/local_position/odom']
        rows = [dict(topic=topic,received_monotonic_ns=100+i,received_ros_ns=200+i,data={})
                for i,topic in enumerate(topics)]
        if reversed_time:
            rows[1]['received_monotonic_ns'] = 90
        payload = ''.join(json.dumps(r)+'\n' for r in rows)
        (root/'events.jsonl').write_text(payload,encoding='utf-8')
        return replay.sha256(root/'events.jsonl')

    def test_missing_or_wrong_provenance_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            digest = self.fixture(root)
            _,_,rows = replay.load_capture(root,digest)
            self.assertEqual([r['ordinal'] for r in rows],list(range(1,8)))
            with self.assertRaisesRegex(ValueError,'capture_hash'):
                replay.load_capture(root,'bad')

    def test_reversed_collector_schedule_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            digest = self.fixture(root,reversed_time=True)
            with self.assertRaisesRegex(ValueError,'reversed_schedule'):
                replay.load_capture(root,digest)

    def test_fc_output_enabled_configuration_is_rejected(self):
        with tempfile.TemporaryDirectory() as folder:
            root = Path(folder)
            digest = self.fixture(root,external=True)
            with self.assertRaisesRegex(ValueError,'external_output'):
                replay.load_capture(root,digest)


if __name__=='__main__':
    unittest.main()
