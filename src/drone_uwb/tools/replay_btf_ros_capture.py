"""1x localhost ROS replay of original capture inputs, with no FC connection.

Collector receipt order is an experimental schedule, not historical B_TF callback
order. One constant translates ROS timestamps to today; all source intervals,
sensor values and RAW monotonic stamps are preserved. No future sensor preload.
Run with PYTHONPATH pointing to an old checkout and --legacy for a control run.
"""
import argparse
from collections import Counter
import gc
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('capture', type=Path)
    p.add_argument('--output', required=True, type=Path)
    p.add_argument('--legacy', action='store_true')
    args = p.parse_args()
    if os.environ.get('ROS_DOMAIN_ID') != '99' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        p.error('requires ROS_DOMAIN_ID=99 and ROS_LOCALHOST_ONLY=1')
    import rclpy
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rosidl_runtime_py.set_message import set_message_fields
    from std_msgs.msg import String
    from sensor_msgs.msg import Imu, Range
    from mavros_msgs.msg import State, TimesyncStatus, ExtendedState
    from nav_msgs.msg import Odometry

    manifest = json.loads((args.capture/'manifest.json').read_text(encoding='utf-8'))
    evidence = next(r for r in manifest['config_evidence'] if r['name'] == 'field-btf.json')
    assert hashlib.sha256(evidence['content_utf8'].encode()).hexdigest() == evidence['sha256']
    config = json.loads(evidence['content_utf8'])
    kinds = {'/uwb/received':String, config['tof_topic']:Range, config['imu_topic']:Imu,
             '/mavros/state':State, '/mavros/extended_state':ExtendedState,
             '/mavros/timesync_status':TimesyncStatus, '/mavros/local_position/odom':Odometry}
    rows = []
    digest = hashlib.sha256()
    with (args.capture/'events.jsonl').open('rb') as src:
        for line in src:
            digest.update(line)
            row = json.loads(line)
            if row.get('topic') in kinds:
                rows.append(row)
    assert all(b['received_monotonic_ns'] >= a['received_monotonic_ns'] for a,b in zip(rows,rows[1:]))
    args.output.mkdir(parents=True, exist_ok=False)
    config_path = args.output/'config.json'
    config_path.write_text(json.dumps(config), encoding='utf-8')
    private = lambda topic: '/btf_replay'+topic
    remapped = list(kinds)+['/uwb/btf_pose','/uwb/btf_xyz','/uwb/btf_status',
                          '/uwb/btf_decision','/uwb/reset_observation_history']
    ros_args = ['--ros-args','-p',f'config_file:={config_path}',
                '-p','require_height_for_pose:=true',
                '-p',f'trace_directory:={args.output}/internal-trace']
    for topic in remapped:
        ros_args += ['-r', f'{topic}:={private(topic)}']
    # Match the real producer/consumer process boundary. Publishing inside the
    # Python solver process would impose its GIL delays on the recorded source.
    node_log = (args.output/'node.txt').open('w', encoding='utf-8')
    process = subprocess.Popen([sys.executable,'-c',
        'from drone_uwb.integration.ros.btf_node import main; main()', *ros_args],
        stdout=node_log, stderr=subprocess.STDOUT)
    rclpy.init(args=[])
    feeder = Node('btf_replay_feeder', use_global_arguments=False)
    observer = Node('btf_replay_observer', use_global_arguments=False)
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    pubs = {topic:feeder.create_publisher(kind, private(topic), qos) for topic,kind in kinds.items()}
    decisions, statuses, failures = [], [], []
    observer.create_subscription(String, private('/uwb/btf_decision'),
        lambda msg:decisions.append(json.loads(msg.data)), qos)
    observer.create_subscription(String, private('/uwb/btf_status'),
        lambda msg:statuses.append(json.loads(msg.data)), qos)
    observer_executor = SingleThreadedExecutor()
    observer_executor.add_node(observer)
    def spin(ex):
        try:
            ex.spin()
        except Exception as exc:
            failures.append(repr(exc))
    threads = [threading.Thread(target=spin,args=(observer_executor,),daemon=True)]
    for thread in threads:
        thread.start()
    sent = Counter()
    max_lateness_ns = 0
    lateness_ns = []
    try:
        deadline = time.monotonic()+5
        while not all(pub.get_subscription_count() for pub in pubs.values()):
            if process.poll() is not None:
                raise RuntimeError('btf_child_exited_before_replay')
            if time.monotonic()>deadline:
                raise RuntimeError('replay_subscriptions_not_matched')
            time.sleep(.02)
        prepared = []
        for row in rows:
            message = kinds[row['topic']]()
            set_message_fields(message, row['data'])
            event = json.loads(message.data) if row['topic'] == '/uwb/received' else None
            prepared.append((message, event))
        # Avoid a full-generation scan of the original capture graph midway
        # through timed playback. Conversion/allocation happens above.
        gc.collect()
        gc_was_enabled = gc.isenabled()
        gc.disable()
        start_mono = time.monotonic_ns()+100_000_000
        shift_ns = time.time_ns()+100_000_000-rows[0]['received_ros_ns']
        for row, (message, event) in zip(rows, prepared):
            target = start_mono+row['received_monotonic_ns']-rows[0]['received_monotonic_ns']
            delay = (target-time.monotonic_ns())/1e9
            if delay>0:
                time.sleep(delay)
            late = time.monotonic_ns()-target
            lateness_ns.append(late)
            max_lateness_ns = max(max_lateness_ns,late)
            topic = row['topic']
            if hasattr(message, 'header'):
                stamp = message.header.stamp
                original = stamp.sec*10**9+stamp.nanosec
                if original > 0:
                    stamp.sec,stamp.nanosec = divmod(original+shift_ns,10**9)
            if event is not None:
                event['host_received_ros_ns'] += shift_ns
                message.data = json.dumps(event)
            pubs[topic].publish(message)
            sent[topic] += 1
            if failures:
                raise RuntimeError(failures)
        time.sleep(.4)
    finally:
        if locals().get('gc_was_enabled', False):
            gc.enable()
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=10.)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait()
            failures.append('btf_child_shutdown_timeout')
        observer_executor.shutdown()
        observer_executor.wake()
        for thread in threads:
            thread.join(3)
        feeder.destroy_node()
        observer.destroy_node()
        rclpy.shutdown()
        node_log.close()
    (args.output/'decisions.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in decisions),encoding='utf-8')
    (args.output/'statuses.jsonl').write_text(''.join(json.dumps(r)+'\n' for r in statuses),encoding='utf-8')
    summary = dict(scope='1x collector-receipt schedule with constant ROS timestamp translation; no FC',
        input_sha256=digest.hexdigest(), legacy=args.legacy, ros_stamp_shift_ns=shift_ns,
        source_duration_s=(rows[-1]['received_monotonic_ns']-rows[0]['received_monotonic_ns'])/1e9,
        max_feeder_lateness_ms=max_lateness_ns/1e6, sent=dict(sent), decision_count=len(decisions),
        feeder_lateness_over_5ms=sum(v>5_000_000 for v in lateness_ns),
        reasons=dict(Counter(r.get('reason') for r in decisions)), failures=failures,
        child_exit_code=process.returncode,
        last_status=statuses[-1] if statuses else None)
    (args.output/'summary.json').write_text(json.dumps(summary,indent=2)+'\n',encoding='utf-8')
    print(json.dumps({k:v for k,v in summary.items() if k != 'last_status'}),flush=True)


if __name__ == '__main__':
    main()
