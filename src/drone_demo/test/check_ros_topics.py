"""Bounded transport check; run manually in local domain 99 after building."""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import rclpy
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import String

from drone_demo.node import require_demo_environment


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mission', action='store_true')
    parser.add_argument('--record-directory', default='')
    args = parser.parse_args()
    if args.record_directory and not args.mission:
        parser.error('--record-directory requires --mission')
    require_demo_environment(os.environ)
    rclpy.init()
    node = rclpy.create_node('demo_transport_check')
    statuses, truths, observations, raw, targets, missions = [], [], [], [], [], []
    subscriptions = [
        node.create_subscription(String, '/demo_status', lambda m: statuses.append(json.loads(m.data)), 10),
        node.create_subscription(PoseStamped, '/demo_pose', truths.append, 10),
        node.create_subscription(PoseWithCovarianceStamped, '/uwb_pose',
                                 observations.append, qos_profile_sensor_data),
        node.create_subscription(String, '/uwb/raw', lambda m: raw.append(json.loads(m.data)),
                                 qos_profile_sensor_data),
    ]
    if args.mission:
        subscriptions.append(node.create_subscription(
            String, '/demo_mission_state', lambda m: missions.append(json.loads(m.data)), 10))
    child = None
    try:
        command = ['ros2', 'launch', 'drone_demo',
                   'mission_demo.launch.py' if args.mission else 'demo.launch.py', 'scenario:=gap']
        if args.record_directory:
            command.append('record_directory:='+args.record_directory)
        child = subprocess.Popen(
            command,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, encoding='utf-8',
            start_new_session=True)
        start = time.monotonic()
        target_subscription = None
        while time.monotonic() - start < 20:
            rclpy.spin_once(node, timeout_sec=0.05)
            if target_subscription is None and time.monotonic() - start > 4:
                target_subscription = node.create_subscription(
                    PoseStamped, '/target_pose', targets.append,
                    QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
                subscriptions.append(target_subscription)
            if child.poll() is not None:
                break
        if child.poll() is None:
            raise RuntimeError('demo launch did not exit within 20 seconds')
        output = child.communicate(timeout=2)[0]
        assert child.returncode == 0, output
        assert statuses and truths and observations and targets and raw, output
        assert all(s['demo'] and not s['z_measured'] for s in statuses)
        assert all(0.2 <= m.pose.position.z <= 2.2 for m in truths)
        assert max(m.pose.position.z for m in truths) - min(m.pose.position.z for m in truths) > 1.0
        assert all(m.pose.pose.position.z == 0 and m.pose.covariance[14] == 1e6 for m in observations)
        assert all(m['demo'] for m in raw)
        assert all(not 160 <= m['seq'] < 200 for m in raw if m['type'] == 'uwb_raw_cycle')
        gaps = [s for s in statuses if 4 <= s['time_s'] < 5]
        assert gaps and all(not s['uwb_observation_published'] for s in gaps)
        assert any(s['time_s'] > 5 and s['uwb_observation_published'] for s in statuses)
        assert statuses[-1]['trajectory_phase'] == 'ARRIVED'
        assert targets[0].pose.position.z == 1.2
        transitions = []
        if args.mission:
            assert missions, output
            previous = None
            for m in missions:
                assert m['demo'] and not m['flight_output']
                if m['state'] != previous:
                    transitions.append(m['state'])
                    previous = m['state']
            for state in ('MOVING', 'APPROACHING', 'SETTLING', 'ARRIVED', 'DEGRADED', 'RECOVERING'):
                assert state in transitions, (state, transitions, output)
            degraded_index = transitions.index('DEGRADED')
            assert 'RECOVERING' in transitions[degraded_index+1:]
            # Shutdown may deliver a last stale-state tick; require actual arrival before exit.
            assert any(m['arrival_valid'] for m in missions)
        print(json.dumps({'result': 'passed', 'domain_id': node.context.get_domain_id(),
                          'truth_messages': len(truths), 'uwb_messages': len(observations),
                          'gap_status_messages': len(gaps), 'late_goal_messages': len(targets),
                          'final_demo_xyz_m': statuses[-1]['truth_xyz_m'],
                          'mission_transitions': transitions,
                          'launch_exit_code': child.returncode}, indent=2))
    finally:
        if child is not None and child.poll() is None:
            os.killpg(child.pid, signal.SIGINT)
            try:
                child.communicate(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(child.pid, signal.SIGKILL)
                child.communicate(timeout=2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
