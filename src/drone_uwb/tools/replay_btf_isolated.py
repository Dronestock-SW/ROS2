"""Separate-process, observation-only B_TF replay; never connects to an FC.

Use an explicitly selected package root for the unchanged A or B subject.
Collector receipt order is a controlled schedule, not historical callback order.
Only header stamps and envelope host_received_ros_ns receive a constant shift.
"""
import argparse
from collections import Counter
import gc
import hashlib
import json
import multiprocessing as mp
import os
from pathlib import Path
import platform
import shutil
import signal
import subprocess
import sys
import time
import traceback

from btf_replay_measurements import read_jsonl, summarize, write_jsonl

CAPTURE_SHA256 = '829d1055c8d7e49b3c4e019560cb5bf4b278b885496b06c030412da5cc30c91e'
OUTPUT_TOPICS = ('/uwb/btf_decision', '/uwb/btf_status', '/uwb/btf_pose')
ALLOWED_NODES = {'btf_replay_feeder', 'btf_replay_observer', 'uwb_btf_node', 'uwb_btf_sensor_ingress'}
READ_ERRORS = {}


def private(topic):
    return '/btf_replay'+topic


def save_json(path, data):
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2, allow_nan=False)+'\n', encoding='utf-8')


def sha256(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024*1024), b''):
            digest.update(block)
    return digest.hexdigest()


def load_capture(capture, expected_sha):
    manifest = json.loads((capture/'manifest.json').read_text(encoding='utf-8'))
    evidence = next(r for r in manifest['config_evidence'] if r['name']=='field-btf.json')
    if hashlib.sha256(evidence['content_utf8'].encode()).hexdigest() != evidence['sha256']:
        raise ValueError('field_config_hash_mismatch')
    config = json.loads(evidence['content_utf8'])
    if config.get('external_output_allowed') is not False:
        raise ValueError('external_output_not_disabled')
    topics = {'/uwb/received', config['imu_topic'], config['tof_topic'], '/mavros/state',
              '/mavros/extended_state', '/mavros/timesync_status', '/mavros/local_position/odom'}
    rows, digest = [], hashlib.sha256()
    with (capture/'events.jsonl').open('rb') as stream:
        for ordinal, line in enumerate(stream, 1):
            digest.update(line)
            row = json.loads(line)
            if row.get('topic') in topics:
                rows.append(dict(row, ordinal=ordinal))
    if digest.hexdigest() != expected_sha:
        raise ValueError('capture_hash_mismatch')
    if not rows or any(b['received_monotonic_ns'] < a['received_monotonic_ns'] for a,b in zip(rows,rows[1:])):
        raise ValueError('empty_or_reversed_schedule')
    if topics != {r['topic'] for r in rows}:
        raise ValueError('missing_input_stream')
    return config, evidence, rows


def identity(row):
    data = row['data']
    result = dict(ordinal=row['ordinal'], topic=row['topic'],
                  source_received_monotonic_ns=row['received_monotonic_ns'],
                  source_received_ros_ns=row['received_ros_ns'])
    if 'header' in data:
        stamp = data['header']['stamp']
        result['source_stamp_ns'] = stamp['sec']*10**9+stamp['nanosec']
    if row['topic']=='/uwb/received':
        envelope = json.loads(data['data'])
        message = envelope['message']
        result.update(type=message['type'],
            source_host_received_ros_ns=envelope['host_received_ros_ns'],
            source_host_received_monotonic_ns=envelope['host_received_monotonic_ns'])
        for key in ('seq','cycle_end_us','boot_id','session','sf_seq','tag_id'):
            if key in message:
                result[key] = message[key]
    else:
        result['type'] = row['topic']
    return result


def shift_message(message, envelope, shift_ns):
    if hasattr(message, 'header'):
        stamp = message.header.stamp
        old = stamp.sec*10**9+stamp.nanosec
        if old>0:
            stamp.sec, stamp.nanosec = divmod(old+shift_ns, 10**9)
    if envelope is not None:
        envelope['host_received_ros_ns'] += shift_ns
        message.data = json.dumps(envelope)


def observer_worker(output, connection):
    """Entry timestamps and payload retention only; encode files after stopping."""
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String
    from geometry_msgs.msg import PoseWithCovarianceStamped
    output = Path(output)
    rows, graphs, errors = [], [], []
    counts, dropped = Counter(), Counter()
    estimated_bytes, node, ready = 0, None, False
    started = time.monotonic_ns()
    def keep(kind, value):
        nonlocal estimated_bytes
        counts[kind] += 1
        cost = sys.getsizeof(value[2])+256
        if estimated_bytes+cost > 128*1024**2 or len(rows)>=100000:
            dropped[kind] += 1
            return
        estimated_bytes += cost
        rows.append(value)
    def receive_string(kind, msg):
        received = time.monotonic_ns()
        keep(kind, (kind,received,msg.data))
    def receive_pose(msg):
        received = time.monotonic_ns()
        p = msg.pose.pose.position
        keep('pose', ('pose',received,(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec,p.x,p.y)))
    try:
        rclpy.init(args=[])
        node = Node('btf_replay_observer', use_global_arguments=False)
        qos = QoSProfile(depth=1000, reliability=ReliabilityPolicy.BEST_EFFORT)
        node.create_subscription(String, private(OUTPUT_TOPICS[0]), lambda m:receive_string('decision',m),qos)
        node.create_subscription(String, private(OUTPUT_TOPICS[1]), lambda m:receive_string('status',m),qos)
        node.create_subscription(PoseWithCovarianceStamped, private(OUTPUT_TOPICS[2]), receive_pose,qos)
        next_graph = 0
        while True:
            if connection.poll() and connection.recv()=='stop':
                break
            rclpy.spin_once(node, timeout_sec=.02)
            now = time.monotonic_ns()
            if now>=next_graph:
                matches = {topic:node.count_publishers(private(topic)) for topic in OUTPUT_TOPICS}
                names = node.get_node_names_and_namespaces()
                graphs.append(dict(monotonic_ns=now,publisher_counts=matches,nodes=names))
                if not ready and all(count==1 for count in matches.values()):
                    connection.send(dict(type='ready',monotonic_ns=now,nodes=names))
                    ready = True
                next_graph = now+1_000_000_000
    except Exception as exc:
        errors.append(repr(exc))
        connection.send(dict(type='error',error=repr(exc)))
    finally:
        stopped = time.monotonic_ns()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        def serializable():
            for kind,received,value in rows:
                if kind=='pose':
                    stamp,x,y = value
                    yield dict(kind=kind,receive_ns=received,stamp_ns=stamp,xy_m=[x,y])
                else:
                    yield dict(kind=kind,receive_ns=received,text=value)
        write_jsonl(output/'observed.jsonl.gz', serializable())
        save_json(output/'observer.json',dict(pid=os.getpid(),started_ns=started,stopped_ns=stopped,
            ready=ready,counts=dict(counts),dropped_by_harness=dict(dropped),errors=errors,
            estimated_retained_bytes=estimated_bytes,graphs=graphs,
            qos=dict(depth=1000,reliability='BEST_EFFORT'),gc_enabled=gc.isenabled()))


def read_text(path):
    try:
        return Path(path).read_text(encoding='utf-8').strip()
    except (OSError, UnicodeError, TypeError) as exc:
        # Some Jetson thermal attributes expose no readable bytes.
        # Missing peripheral readings must be explicit, not fabricated as zero.
        READ_ERRORS[str(path)] = repr(exc)
        return None


def process_sample(pid):
    def task(path):
        stat = read_text(path/'stat')
        if stat is None:
            return None
        fields = stat[stat.rfind(')')+2:].split()
        status = read_text(path/'status') or ''
        wanted = ('VmRSS:', 'voluntary_ctxt_switches:', 'nonvoluntary_ctxt_switches:')
        return dict(state=fields[0],utime_ticks=int(fields[11]),stime_ticks=int(fields[12]),
            starttime_ticks=int(fields[19]),rss_pages=int(fields[21]),
            status={line.split(':',1)[0]:line.split(':',1)[1].strip()
                    for line in status.splitlines() if line.startswith(wanted)})
    root = Path('/proc')/str(pid)
    return dict(process=task(root),threads={p.name:task(p) for p in (root/'task').glob('*')})


def sampler_worker(output, pids, connection):
    output = Path(output)
    samples, errors = [], []
    temperatures = list(Path('/sys/class/thermal').glob('thermal_zone*/temp'))
    frequencies = list(Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_cur_freq'))
    governors = list(Path('/sys/devices/system/cpu').glob('cpu[0-9]*/cpufreq/scaling_governor'))
    next_sample = time.monotonic()
    try:
        while True:
            READ_ERRORS.clear()
            begun = time.monotonic_ns()
            record = dict(monotonic_ns=begun,realtime_ns=time.time_ns(),
                loadavg=read_text('/proc/loadavg'),cpu_stat=read_text('/proc/stat'),
                meminfo=read_text('/proc/meminfo'),disk_free_bytes=shutil.disk_usage(output).free,
                processes={str(pid):process_sample(pid) for pid in pids},
                temperatures={str(p):read_text(p) for p in temperatures},
                cpu_frequencies={str(p):read_text(p) for p in frequencies},
                cpu_governors={str(p):read_text(p) for p in governors})
            record['unavailable_reads'] = dict(READ_ERRORS)
            record['sample_cost_ns'] = time.monotonic_ns()-begun
            samples.append(record)
            next_sample += 1.
            if connection.poll(max(0.,next_sample-time.monotonic())):
                if connection.recv()=='stop':
                    break
    except Exception as exc:
        errors.append(repr(exc))
    finally:
        write_jsonl(output/'resources.jsonl.gz',samples)
        save_json(output/'sampler.json',dict(errors=errors,count=len(samples),
            temperature_sensor_types={str(p):read_text(p.with_name('type')) for p in temperatures},
            clock_ticks_per_second=os.sysconf('SC_CLK_TCK'),page_bytes=os.sysconf('SC_PAGE_SIZE')))


def stop_process(process, failures, label):
    if process is None or process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=10)
    except subprocess.TimeoutExpired:
        failures.append(label+'_shutdown_timeout')
        process.kill()
        process.wait(timeout=5)


def stop_worker(process, connection, failures, label):
    if process is None:
        return
    if process.is_alive():
        connection.send('stop')
    process.join(timeout=45)
    if process.is_alive():
        failures.append(label+'_shutdown_timeout')
        process.kill()
        process.join(timeout=5)
    if process.exitcode != 0:
        failures.append(label+'_exit_'+str(process.exitcode))


def subject_entry(package, lock_path, output, ros_args):
    """Verify imports inside the actual subject process, then call its main."""
    import drone_uwb.integration.ros.btf_node as entry
    package = Path(package).resolve()
    expected = json.loads(Path(lock_path).read_text(encoding='utf-8'))['source_hashes']
    imported = {}
    for name,module in list(sys.modules.items()):
        if name=='drone_uwb' or name.startswith('drone_uwb.'):
            source = Path(module.__file__).resolve()
            relative = source.relative_to(package).as_posix()
            actual = sha256(source)
            if expected.get(relative)!=actual:
                raise ValueError('imported_subject_source_mismatch:'+name)
            imported[name] = dict(path=str(source),sha256=actual)
    save_json(Path(output)/'imported-source.json',dict(pid=os.getpid(),modules=imported))
    entry.main(args=ros_args)
    return 0


def run(args):
    if os.environ.get('ROS_DOMAIN_ID')!='99' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
        raise ValueError('requires_ROS_DOMAIN_ID_99_and_ROS_LOCALHOST_ONLY_1')
    if sys.platform!='linux':
        raise ValueError('run_requires_Linux_host')
    package = args.package_root.resolve()
    if not (package/'drone_uwb/integration/ros/btf_node.py').is_file():
        raise ValueError('not_a_drone_uwb_package_root')
    config, evidence, rows = load_capture(args.capture,args.expected_sha256)
    output = args.output.resolve()
    output.mkdir(parents=True,exist_ok=False)
    save_json(output/'config.json',config)
    if shutil.disk_usage(output).free < (256+args.output_budget_mib)*1024**2:
        raise ValueError('insufficient_disk_before_run')
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy
    from rclpy.utilities import get_rmw_implementation_identifier
    from rosidl_runtime_py.set_message import set_message_fields
    from std_msgs.msg import String
    from sensor_msgs.msg import Imu, Range
    from mavros_msgs.msg import State, ExtendedState, TimesyncStatus
    from nav_msgs.msg import Odometry
    kinds = {'/uwb/received':String,config['imu_topic']:Imu,config['tof_topic']:Range,
        '/mavros/state':State,'/mavros/extended_state':ExtendedState,
        '/mavros/timesync_status':TimesyncStatus,'/mavros/local_position/odom':Odometry}
    expected = Counter(row['topic'] for row in rows)
    prepared, identities = [], []
    for row in rows:
        message = kinds[row['topic']]()
        set_message_fields(message,row['data'])
        envelope = json.loads(message.data) if row['topic']=='/uwb/received' else None
        prepared.append((message,envelope))
        identities.append(identity(row))
    source_hashes = {str(p.relative_to(package)):sha256(p)
                     for p in sorted((package/'drone_uwb').rglob('*.py'))}
    lock = json.loads(args.source_lock.read_text(encoding='utf-8'))
    if lock.get('revision') != args.revision or lock.get('source_hashes') != source_hashes:
        raise ValueError('subject_source_lock_mismatch')
    manifest = dict(schema=1,label=args.label,revision_label=args.revision,
        phase='prepared',input_sha256=args.expected_sha256,field_config_sha256=evidence['sha256'],
        package_root=str(package),source_hashes=source_hashes,python=sys.version,platform=platform.platform(),
        source_lock_sha256=sha256(args.source_lock),
        rmw=get_rmw_implementation_identifier(),environment={k:os.environ.get(k) for k in
            ('ROS_DOMAIN_ID','ROS_LOCALHOST_ONLY','RMW_IMPLEMENTATION','ROS_DISTRO','OMP_NUM_THREADS','OPENBLAS_NUM_THREADS')},
        tools={p.name:sha256(p) for p in (Path(__file__),Path(__file__).with_name('btf_replay_measurements.py'))},
        expected_input_counts=dict(expected),source_duration_s=(rows[-1]['received_monotonic_ns']-rows[0]['received_monotonic_ns'])/1e9,
        prefeed_observation_s=args.prefeed_seconds,drain_s=args.drain_seconds,
        output_budget_mib=args.output_budget_mib,
        source_schedule='collector received_monotonic_ns, full selected inputs at 1x',
        timestamp_translation='one constant on nonzero header stamps and envelope host_received_ros_ns only',
        preserved_fields=['source monotonic times','sensor values','RAW sensor times','timesync remote timestamp and offset'],
        feeder_gc_during_feed=False,observer_gc_during_feed=True,
        start_graph=None,end_graph=None,failures=[])
    save_json(output/'manifest.json',manifest)
    env = dict(os.environ)
    env['PYTHONPATH'] = str(package)+os.pathsep+env.get('PYTHONPATH','')
    ros_args = ['--ros-args','-p',f'config_file:={output}/config.json',
                '-p','require_height_for_pose:=true','-p',f'trace_directory:={output}/internal-trace']
    for topic in [*kinds,*OUTPUT_TOPICS,'/uwb/btf_xyz','/uwb/reset_observation_history']:
        ros_args += ['-r',f'{topic}:={private(topic)}']
    manifest['subject_command'] = [sys.executable,str(Path(__file__).resolve()),'--subject-entry',
        str(package),str(args.source_lock.resolve()),str(output),*ros_args]
    ctx = mp.get_context('spawn')
    observe_parent,observe_child = ctx.Pipe()
    sample_parent,sample_child = ctx.Pipe()
    subject = observer = sampler = feeder = node_log = None
    failures = manifest['failures']
    actual, start_ns, feed_end_ns, window_end_ns = [], None, None, None
    gc_was_enabled = gc.isenabled()
    try:
        rclpy.init(args=[])
        feeder = Node('btf_replay_feeder',use_global_arguments=False)
        qos = QoSProfile(depth=100,reliability=ReliabilityPolicy.BEST_EFFORT)
        pubs = {topic:feeder.create_publisher(kind,private(topic),qos) for topic,kind in kinds.items()}
        observer = ctx.Process(target=observer_worker,args=(str(output),observe_child))
        observer.start()
        node_log = (output/'subject.log').open('w',encoding='utf-8')
        subject = subprocess.Popen(manifest['subject_command'],env=env,stdout=node_log,
                                   stderr=subprocess.STDOUT,start_new_session=True)
        deadline,observer_ready = time.monotonic()+15.,False
        while True:
            if subject.poll() is not None or not observer.is_alive():
                raise RuntimeError('child_exited_before_replay')
            if observe_parent.poll():
                message = observe_parent.recv()
                if message['type']=='error':
                    raise RuntimeError(message['error'])
                observer_ready = message['type']=='ready'
            counts = {topic:pub.get_subscription_count() for topic,pub in pubs.items()}
            if any(v>1 for v in counts.values()):
                raise RuntimeError('unexpected_input_subscriber')
            if observer_ready and all(v==1 for v in counts.values()):
                break
            if time.monotonic()>deadline:
                raise RuntimeError('publisher_subscriber_match_timeout')
            time.sleep(.02)
        background = subprocess.run(['ps','-eo','pid,comm,pcpu','--sort=-pcpu'],
                                    capture_output=True,text=True,check=True).stdout.splitlines()
        manifest['background_process_snapshot'] = background
        background_pids = [int(line.split()[0]) for line in background[1:13]]
        tracked_pids = sorted(set([os.getpid(),subject.pid,observer.pid,*background_pids]))
        manifest['resource_sample_pids'] = tracked_pids
        sampler = ctx.Process(target=sampler_worker,args=(str(output),tracked_pids,sample_child))
        sampler.start()
        manifest['pids'] = dict(feeder=os.getpid(),subject=subject.pid,observer=observer.pid,sampler=sampler.pid)
        # Fixed prefeed observation; source timesync is NOT preloaded here.
        wait_until = time.monotonic()+args.prefeed_seconds
        while time.monotonic()<wait_until:
            if subject.poll() is not None or not observer.is_alive() or not sampler.is_alive():
                raise RuntimeError('child_exited_during_prefeed')
            time.sleep(min(.1,max(0.,wait_until-time.monotonic())))
        manifest['start_graph'] = feeder.get_node_names_and_namespaces()
        if any(name not in ALLOWED_NODES or namespace!='/' for name,namespace in manifest['start_graph']):
            raise RuntimeError('unexpected_domain_participant')
        if any(pub.get_subscription_count()!=1 for pub in pubs.values()):
            raise RuntimeError('input_match_changed_before_feed')
        gc.collect()
        gc.disable()
        # Translate every message before timing starts, using one clock anchor.
        # This keeps JSON serialization out of the publish schedule.
        anchor_mono_before,anchor_ros = time.monotonic_ns(),time.time_ns()
        anchor_mono_after = time.monotonic_ns()
        anchor_mono = (anchor_mono_before+anchor_mono_after)//2
        start_ns = anchor_mono+5_000_000_000
        shift_ns = anchor_ros+5_000_000_000-rows[0]['received_ros_ns']
        for message,envelope in prepared:
            shift_message(message,envelope,shift_ns)
        if time.monotonic_ns()>=start_ns-100_000_000:
            raise RuntimeError('message_translation_exceeded_start_margin')
        manifest.update(ros_shift_ns=shift_ns,anchor_mono_ns=anchor_mono,anchor_ros_ns=anchor_ros,
                        anchor_clock_pair_width_ns=anchor_mono_after-anchor_mono_before,
                        start_ns=start_ns,phase='feeding')
        save_json(output/'manifest.json',manifest)
        next_guard = start_ns
        for row,(message,envelope),ident in zip(rows,prepared,identities):
            target = start_ns+row['received_monotonic_ns']-rows[0]['received_monotonic_ns']
            delay = (target-time.monotonic_ns())/1e9
            if delay>0:
                time.sleep(delay)
            before_ros_mono = time.monotonic_ns()
            before_ros = time.time_ns()
            before = time.monotonic_ns()
            returned = False
            try:
                pubs[row['topic']].publish(message)
                returned = True
            finally:
                after = time.monotonic_ns()
                actual.append(dict(ident,target_ns=target,before_ns=before,after_ns=after,
                                   before_ros_ns=before_ros,ros_shift_ns=shift_ns,
                                   clock_pair_width_ns=before-before_ros_mono,
                                   publish_returned=returned))
            if after>=next_guard:
                if subject.poll() is not None or not observer.is_alive() or not sampler.is_alive():
                    raise RuntimeError('child_exited_during_feed')
                next_guard = after+1_000_000_000
        feed_end_ns = time.monotonic_ns()
        window_end_ns = feed_end_ns+round(args.drain_seconds*1e9)
        while time.monotonic_ns()<window_end_ns:
            time.sleep(min(.05,max(0.,(window_end_ns-time.monotonic_ns())/1e9)))
        manifest['end_graph'] = feeder.get_node_names_and_namespaces()
        manifest['phase'] = 'feed_completed'
    except Exception as exc:
        failures.append(repr(exc))
        manifest['exception'] = traceback.format_exc()
        manifest['phase'] = 'failed'
    finally:
        if gc_was_enabled:
            gc.enable()
        if feed_end_ns is None:
            feed_end_ns = time.monotonic_ns()
        if window_end_ns is None:
            window_end_ns = feed_end_ns
        stop_process(subject,failures,'subject')
        stop_worker(observer,observe_parent,failures,'observer')
        stop_worker(sampler,sample_parent,failures,'sampler')
        if feeder is not None:
            feeder.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        if node_log is not None:
            node_log.close()
        manifest.update(feed_end_ns=feed_end_ns,observation_end_ns=window_end_ns,
            subject_exit_code=subject.returncode if subject is not None else None,
            completed_input_counts=dict(Counter(r['topic'] for r in actual if r['publish_returned'])),
            final_free_bytes=shutil.disk_usage(output).free)
        if manifest['completed_input_counts'] != manifest['expected_input_counts']:
            failures.append('input_count_mismatch')
        if subject is not None and subject.returncode != 0:
            failures.append('subject_exit_'+str(subject.returncode))
        if not (output/'imported-source.json').is_file():
            failures.append('subject_import_verification_missing')
        for label in ('observer','sampler'):
            path = output/(label+'.json')
            if not path.is_file():
                failures.append(label+'_metadata_missing')
                continue
            metadata = json.loads(path.read_text(encoding='utf-8'))
            if metadata.get('errors') or metadata.get('dropped_by_harness'):
                failures.append(label+'_reported_error_or_drop')
            if label=='observer' and start_ns is not None:
                for graph in metadata['graphs']:
                    if not start_ns<=graph['monotonic_ns']<=window_end_ns:
                        continue
                    if any(count!=1 for count in graph['publisher_counts'].values()):
                        failures.append('output_publisher_match_changed')
                        break
                    if any(name not in ALLOWED_NODES or ns!='/' for name,ns in graph['nodes']):
                        failures.append('unexpected_domain_participant_during_observation')
                        break
        if manifest['end_graph'] and any(name not in ALLOWED_NODES or ns!='/'
                                        for name,ns in manifest['end_graph']):
            failures.append('unexpected_domain_participant_at_end')
        write_jsonl(output/'feeder.jsonl.gz',actual)
        if start_ns is not None and start_ns<=feed_end_ns and (output/'observed.jsonl.gz').is_file():
            result,links = summarize(actual,read_jsonl(output/'observed.jsonl.gz'),
                start_ns=start_ns,feed_end_ns=feed_end_ns,observation_end_ns=window_end_ns)
            write_jsonl(output/'links.jsonl.gz',links)
            result['run_failures'] = failures
            offsets = [(r['before_ros_ns']-r['before_ns'])-(manifest['anchor_ros_ns']-manifest['anchor_mono_ns'])
                       for r in actual]
            source_offsets = [(r['source_received_ros_ns']-r['source_received_monotonic_ns'])
                              -(actual[0]['source_received_ros_ns']-actual[0]['source_received_monotonic_ns'])
                              for r in actual]
            result['clock_consistency'] = dict(
                system_minus_monotonic_change_from_anchor_ns=dict(min=min(offsets,default=None),max=max(offsets,default=None)),
                original_collector_clock_offset_change_ns=dict(min=min(source_offsets,default=None),max=max(source_offsets,default=None)),
                max_publish_clock_pair_width_ns=max((r['clock_pair_width_ns'] for r in actual),default=None),
                anchor_clock_pair_width_ns=manifest['anchor_clock_pair_width_ns'],
                interpretation='Signed offset changes include clock-read uncertainty. Original capture drift is separate.')
            save_json(output/'summary.json',result)
        if failures:
            manifest['phase'] = 'failed'
        save_json(output/'manifest.json',manifest)
    print(json.dumps(dict(output=str(output),phase=manifest['phase'],failures=failures)),flush=True)
    return 1 if failures else 0


def main():
    if len(sys.argv)>1 and sys.argv[1]=='--subject-entry':
        return subject_entry(*sys.argv[2:5],sys.argv[5:])
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture',type=Path)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--package-root',type=Path,required=True)
    parser.add_argument('--label',required=True)
    parser.add_argument('--revision',required=True,help='provenance label; source_hashes are authoritative')
    parser.add_argument('--source-lock',type=Path,required=True,help='revision and expected source_hashes JSON')
    parser.add_argument('--expected-sha256',default=CAPTURE_SHA256)
    parser.add_argument('--prefeed-seconds',type=float,default=60.)
    parser.add_argument('--drain-seconds',type=float,default=2.)
    parser.add_argument('--output-budget-mib',type=int,default=160)
    args = parser.parse_args()
    if args.prefeed_seconds<0 or args.drain_seconds<=0 or args.output_budget_mib<1:
        parser.error('invalid_timing_or_storage_budget')
    return run(args)


if __name__=='__main__':
    raise SystemExit(main())
