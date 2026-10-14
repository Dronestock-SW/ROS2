"""Linux/ROS smoke test with a synthetic source and a synthetic subject only.

No original flight replay and no production B_TF algorithm are executed here.
The subject only echoes a known pose to the private test output topics.
"""
import hashlib
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time
import unittest

SUBJECT = '''import json
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import Imu, Range
from mavros_msgs.msg import State, ExtendedState, TimesyncStatus
from nav_msgs.msg import Odometry

def main(args=None):
    rclpy.init(args=args)
    node = Node('uwb_btf_node', automatically_declare_parameters_from_overrides=True)
    qos = qos_profile_sensor_data
    decision = node.create_publisher(String, '/uwb/btf_decision', qos)
    status = node.create_publisher(String, '/uwb/btf_status', qos)
    pose_pub = node.create_publisher(PoseWithCovarianceStamped, '/uwb/btf_pose', qos)
    def raw(msg):
        event = json.loads(msg.data)
        sample = event['message']
        pose = PoseWithCovarianceStamped()
        stamp = event['host_received_ros_ns']
        pose.header.stamp.sec,pose.header.stamp.nanosec = divmod(stamp,10**9)
        pose.pose.pose.position.x,pose.pose.pose.position.y = 1.25,2.5
        pose_pub.publish(pose)
        decision.publish(String(data=json.dumps(dict(seq=sample['seq'],
            source_cycle_end_us=sample['cycle_end_us'], stamp_ns=stamp,
            xy_m=[1.25,2.5], reason='synthetic_echo', published=True, processing_ms=0.))))
    node.create_subscription(String, '/uwb/received', raw, qos)
    for kind,topic in [(Imu,'/imu'),(Range,'/tof'),(State,'/mavros/state'),
                       (ExtendedState,'/mavros/extended_state'),
                       (TimesyncStatus,'/mavros/timesync_status'),(Odometry,'/mavros/local_position/odom')]:
        node.create_subscription(kind,topic,lambda msg:None,qos)
    node.create_timer(.05,lambda:status.publish(String(data='{"synthetic_subject": true}')))
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt,rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
'''


@unittest.skipUnless(sys.platform=='linux' and importlib.util.find_spec('rclpy') is not None,
                     'requires Linux with ROS; pure offline tests run elsewhere')
class ReplayRosHarnessTest(unittest.TestCase):
    def test_full_harness_with_synthetic_subject(self):
        self.assertEqual(os.environ.get('ROS_DOMAIN_ID'),'99')
        self.assertEqual(os.environ.get('ROS_LOCALHOST_ONLY'),'1')
        from rosidl_runtime_py.convert import message_to_ordereddict
        from sensor_msgs.msg import Imu,Range
        from mavros_msgs.msg import State,ExtendedState,TimesyncStatus
        from nav_msgs.msg import Odometry
        # Persist on request so a test failure retains child logs and metadata.
        parent = os.environ.get('BTF_HARNESS_TEST_OUTPUT')
        temporary = tempfile.TemporaryDirectory(prefix='btf-harness-') if not parent else None
        root = Path(temporary.name) if temporary else Path(parent)
        root.mkdir(parents=True,exist_ok=True)
        try:
            package = root/'synthetic-package'
            leaf = package/'drone_uwb/integration/ros'
            leaf.mkdir(parents=True)
            for folder in (package/'drone_uwb',package/'drone_uwb/integration',leaf):
                (folder/'__init__.py').write_text('',encoding='utf-8')
            (leaf/'btf_node.py').write_text(SUBJECT,encoding='utf-8')
            source_lock = root/'source-lock.json'
            source_lock.write_text(json.dumps(dict(revision='synthetic_only',source_hashes={
                str(p.relative_to(package)):hashlib.sha256(p.read_bytes()).hexdigest()
                for p in sorted((package/'drone_uwb').rglob('*.py'))})),encoding='utf-8')
            capture = root/'synthetic-capture'
            capture.mkdir()
            config_text = json.dumps(dict(external_output_allowed=False,imu_topic='/imu',tof_topic='/tof'))
            evidence = dict(name='field-btf.json',content_utf8=config_text,
                            sha256=hashlib.sha256(config_text.encode()).hexdigest())
            (capture/'manifest.json').write_text(json.dumps(dict(config_evidence=[evidence])),encoding='utf-8')
            topics = [('/imu',Imu()),('/tof',Range()),('/mavros/state',State()),
                ('/mavros/extended_state',ExtendedState()),('/mavros/timesync_status',TimesyncStatus()),
                ('/mavros/local_position/odom',Odometry())]
            mono,ros = time.monotonic_ns(),time.time_ns()
            rows = []
            for index,(topic,message) in enumerate(topics):
                received_ros = ros+index*20_000_000
                if hasattr(message,'header'):
                    message.header.stamp.sec,message.header.stamp.nanosec = divmod(received_ros-1_000_000,10**9)
                rows.append(dict(topic=topic,data=message_to_ordereddict(message),
                    received_monotonic_ns=mono+index*20_000_000,received_ros_ns=received_ros))
            envelope = dict(host_received_ros_ns=ros+119_000_000,host_received_monotonic_ns=mono+119_000_000,
                            message=dict(type='uwb_raw_cycle',seq=7,cycle_end_us=99))
            rows.append(dict(topic='/uwb/received',data=dict(data=json.dumps(envelope)),
                             received_ros_ns=ros+120_000_000,received_monotonic_ns=mono+120_000_000))
            source = ''.join(json.dumps(row)+'\n' for row in rows).encode()
            (capture/'events.jsonl').write_bytes(source)
            runner = Path(__file__).parents[1]/'tools/replay_btf_isolated.py'
            command = [sys.executable,str(runner),str(capture),'--output',str(root/'run'),
                '--package-root',str(package),'--label','synthetic_harness_test','--revision','synthetic_only',
                '--source-lock',str(source_lock),
                '--expected-sha256',hashlib.sha256(source).hexdigest(),'--prefeed-seconds','0',
                '--drain-seconds','.3','--output-budget-mib','1']
            result = subprocess.run(command,capture_output=True,text=True,timeout=60)
            (root/'test-stdout.txt').write_text(result.stdout+'\n'+result.stderr,encoding='utf-8')
            self.assertEqual(result.returncode,0,result.stdout+'\n'+result.stderr)
            summary = json.loads((root/'run/summary.json').read_text(encoding='utf-8'))
            manifest = json.loads((root/'run/manifest.json').read_text(encoding='utf-8'))
            self.assertEqual(summary['run_failures'],[])
            self.assertEqual(summary['attempted_count'],7)
            self.assertEqual(summary['pose_mapping'],{'matched':1})
            self.assertEqual(summary['reasons'],{'synthetic_echo':1})
            self.assertEqual(summary['raw_to_pose_ms']['count'],1)
            self.assertEqual(manifest['expected_input_counts'],manifest['completed_input_counts'])
            self.assertEqual(len(set(manifest['pids'].values())),4)
            self.assertEqual(manifest['phase'],'feed_completed')
            imported = json.loads((root/'run/imported-source.json').read_text(encoding='utf-8'))
            self.assertEqual(imported['pid'],manifest['pids']['subject'])
            self.assertTrue(imported['modules']['drone_uwb.integration.ros.btf_node']['path'].startswith(str(package)))
        finally:
            if temporary:
                temporary.cleanup()


if __name__=='__main__':
    unittest.main()
