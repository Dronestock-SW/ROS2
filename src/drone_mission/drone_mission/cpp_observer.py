"""Read-only sensor/library and web-contract adapter for the C++ executor.

No ARM, mode, LAND or setpoint writes exist in this node. PX4 message interval
requests and parameter reads retain the existing sensor transport setup.
"""
from dataclasses import asdict, replace
import copy
import hashlib
import json
import math
import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import String
from mavros_msgs.msg import State, ExtendedState, EstimatorStatus, RCIn
from mavros_msgs.srv import CommandBool, CommandInt, CommandLong, CommandTOL, SetMode
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from sensor_msgs.msg import BatteryState, PointCloud
from .node import FlightNode
from .contracts import Settings
from .local_xy import LocalXYMission
from .mission_chain import MissionChain
from .anchor_trial import validate_anchor_trial
from .preflight import field_preflight
from mavros_msgs.msg import TimesyncStatus
from .recording import AsyncEventRecorder, flight_sample, sensor_sources
from .persistence import RequestJournal
from .observation_policy import observation_status
from .operator_arm import arm_request
from datetime import datetime


def clean(value):
    if isinstance(value, (tuple, list)): return [clean(v) for v in value]
    if isinstance(value, dict): return {k: clean(v) for k, v in value.items()}
    if type(value) is float and not math.isfinite(value): return None
    return value


class GraphProbe:
    def __init__(self, node): self.node = node
    def get_subscription_count(self):
        return self.node.count_subscribers('/mavros/setpoint_raw/local')


class CppObserver(Node):
    # Share the proven sensor transforms; never inherit the flight constructor.
    receive = FlightNode.receive
    age = FlightNode.age
    snapshot = FlightNode.snapshot
    bridge = FlightNode.bridge
    query = FlightNode.query
    parameters_received = FlightNode.parameters_received

    def __init__(self):
        super().__init__('cpp_field_observer')
        self.declare_parameter('config_file', '')
        self.declare_parameter('execute', False)
        self.declare_parameter('test_mode', False)
        self.declare_parameter('record_directory', '')
        self.declare_parameter('ledger_file', str(Path.home()/'.local/state/dronestock-flight/requests.json'))
        self.settings = replace(Settings(**json.loads(Path(self.get_parameter('config_file').value).read_text(encoding='utf-8'))), execute=self.get_parameter('execute').value)
        if not self.settings.local_xy or self.settings.mission_ai != 'cpp_bt':
            raise ValueError('cpp_observer_requires_cpp_local_xy')
        test = self.get_parameter('test_mode').value
        domain = self.context.get_domain_id()
        if not ((test and domain == 99 and os.environ.get('ROS_LOCALHOST_ONLY') == '1') or (not test and domain == (1 if self.settings.drone_id == '5' else 2))):
            raise ValueError('invalid_domain_or_local_test_mode')
        self.rc_required = not test
        self.ledger = Path(self.get_parameter('ledger_file').value)
        consumed = json.loads(self.ledger.read_text(encoding='utf-8'))['consumed'] if self.ledger.exists() else []
        self.session = LocalXYMission(self.settings, consumed)
        self.journal = RequestJournal(self.ledger, consumed)
        self.samples = {}; self.origin = None
        self.param_value = None; self.param_received = self.param_requested = float('-inf'); self.param_future = None
        self.streams_requested = self.feedback_stream_requested = False; self.origin_requested = float('-inf')
        self.local_pub = GraphProbe(self)
        # Clients below only provide graph readiness to snapshot(). No flight
        # request method is exposed or called by this adapter.
        self.command_clients = {name: self.create_client(cls, topic) for name, cls, topic in (
            ('arm', CommandBool, '/mavros/cmd/arming'), ('land', CommandTOL, '/mavros/cmd/land'),
            ('takeoff', CommandTOL, '/mavros/cmd/takeoff'), ('reposition', CommandInt, '/mavros/cmd/command_int'),
            ('offboard_mode', SetMode, '/mavros/set_mode'))}
        self.param_client = self.create_client(GetParameters, '/mavros/param/get_parameters')
        self.stream_client = self.create_client(CommandLong, '/mavros/cmd/command')
        for key, cls, topic in (
            ('state', State, '/mavros/state'), ('landed', ExtendedState, '/mavros/extended_state'),
            ('estimator', EstimatorStatus, '/mavros/estimator_status'), ('rc', RCIn, '/mavros/rc/in'),
            ('pose', Odometry, '/mavros/local_position/odom'), ('uwb', PoseWithCovarianceStamped, '/uwb/btf_pose'),
            ('height', PoseStamped, '/uwb/btf_xyz'), ('battery', BatteryState, '/mavros/battery'),
            ('lidar', PointCloud, '/point_cloud')):
            if key == 'lidar' and self.settings.lidar_gap.get('enabled') is not True: continue
            if key == 'lidar':
                self.create_subscription(cls,topic,self.lidar_received,qos_profile_sensor_data)
                continue
            self.create_subscription(cls, topic, lambda m, k=key: self.receive(k, m), qos_profile_sensor_data)
        self.create_subscription(String, self.settings.bridge_topic, self.bridge, 10)
        self.create_subscription(TimesyncStatus,'/mavros/timesync_status',self.timesync_received,qos_profile_sensor_data)
        self.create_subscription(String, '/mission/assignment', self.assignment, 10)
        self.create_subscription(String, '/flight_state', self.native_status, qos_profile_sensor_data)
        self.output = self.create_publisher(String, '/flight/cpp_input', 1)
        self.valid_pub = self.create_publisher(String, '/target_valid', 10)
        self.native_session = None; self.forwarded = None; self.previous_phase = None
        directory = self.get_parameter('record_directory').value
        if self.settings.execute and not directory: raise ValueError('record_directory_required')
        self.log = None
        if directory:
            root = Path(directory); root.mkdir(parents=True, exist_ok=False)
            self.log = (root/'observer-events.jsonl').open('x', encoding='utf-8')
        self.recorder = AsyncEventRecorder(self.log, sync_interval_s=1.) if self.log else None
        self.record_fault = False
        self.create_timer(.1, self.tick)
        self.create_timer(1., self.query)
        self.up_limit = None; self.up_limit_received = float('-inf'); self.up_future = None; self.up_requested = float('-inf')
        self.create_timer(1., self.query_up_limit)

    def lidar_received(self, msg):
        self.receive('lidar',msg)
        # Relay a new acquisition immediately. Waiting for a 100ms metadata
        # timer can consume the entire original 150ms scan freshness budget.
        if self.native_session is not None: self.tick()

    def query_up_limit(self):
        if self.up_future is not None and time.monotonic()-self.up_requested > 2.:
            self.param_client.remove_pending_request(self.up_future); self.up_future = None
        if self.up_future is not None or not self.param_client.service_is_ready(): return
        self.up_requested = time.monotonic()
        self.up_future = self.param_client.call_async(GetParameters.Request(names=['MPC_Z_VEL_MAX_UP']))
        def completed(future):
            if future is not self.up_future: return
            self.up_future = None
            try:
                value = future.result().values[0]
                self.up_limit = value.double_value if value.type == 3 else value.integer_value if value.type == 2 else None
                self.up_limit_received = time.monotonic()
            except Exception: self.up_limit = None
        self.up_future.add_done_callback(completed)

    def native_status(self, msg):
        value = json.loads(msg.data)
        if value.get('executor_backend') != 'sangwon_cpp_executor': return
        self.native_session = value['executor_session']
        self.last_native_status = value
        phase = value['phase']
        self.session.phase = phase
        if phase != self.previous_phase:
            self.record(dict(type='native_transition', value=value)); self.previous_phase = phase

    def timesync_received(self,msg):
        self.samples['timesync']=(dict(remote_timestamp_ns=msg.remote_timestamp_ns,
            estimated_offset_ns=msg.estimated_offset_ns,round_trip_time_ms=msg.round_trip_time_ms),time.monotonic())

    def record(self, value):
        if self.recorder and not self.recorder.submit(dict(value, wall_ns=time.time_ns(), monotonic_ns=time.monotonic_ns())):
            self.record_fault = True

    def assignment(self, msg):
        try:
            if len(msg.data) > 65536: raise ValueError('assignment_too_large')
            payload = json.loads(msg.data)
            if payload.get('control_action') == 'arm': raise ValueError('native_trial_requires_combined_start')
            if payload.get('control_action') in ('takeoff','takeoff_land') or (payload.get('control_action') == 'land' and not self.session.assignment and self.settings.start_mode == 'takeoff_hold'):
                wire_action = payload['control_action']
                action = 'land' if wire_action == 'takeoff_land' else wire_action
                key = payload.get('control_request_id')
                if key in self.session.consumed:
                    self.session.last_web_s = time.monotonic()
                    return
                key = arm_request(payload,self.settings,self.native_session,time.time(),action=wire_action)
                if self.settings.start_mode != 'takeoff_hold':raise ValueError('separate_takeoff_disabled')
                native = getattr(self,'last_native_status',{}).get('native_takeoff',{})
                if not native.get('can_takeoff' if action == 'takeoff' else 'can_land'):raise ValueError('native_takeoff_or_land_not_ready')
                ceiling = payload.get('ceiling_height_m')
                if action == 'takeoff' and (type(ceiling) not in (float,int) or not math.isfinite(ceiling) or ceiling < self.settings.expected_mis_takeoff_alt_m+.25):raise ValueError('takeoff_ceiling_clearance_required')
                self.session.intent = dict(action=action,request_id=key,requested_at=payload['control_requested_at'],
                    requested_s=datetime.fromisoformat(payload['control_requested_at'].replace('Z','+00:00')).timestamp())
                self.session.cpp_ceiling = ceiling
                self.session.last_web_s = time.monotonic()
                self.session.validation = dict(accepted=True,reason='request_valid',request_id=key)
                return
            if payload.get('control_action') == 'start' and not self.session.assignment:
                launch = getattr(self,'last_native_status',{}).get('mission_start_xy_m') if self.settings.start_mode == 'takeoff_hold' else self.snapshot().xy
                validate_anchor_trial(payload, launch, self.settings, time.time())
            self.session.submit(payload, time.monotonic(), time.time())
        except (ValueError, TypeError, KeyError) as error:
            self.session.validation = dict(accepted=False, reason=str(error))
        self.valid_pub.publish(String(data=json.dumps(self.session.validation)))

    def tick(self):
        # Disk delay/failure cannot stop ROS input reception and publication.
        recording_ok = bool(self.recorder) and not self.record_fault and not self.recorder.error and not self.journal.worker.error
        s = self.snapshot(); now = time.monotonic()
        if now-getattr(self,'last_evidence_s',0) >= (.2 if s.armed else 1.):
            self.last_evidence_s=now
            native=getattr(self,'last_native_status',{})
            self.record(flight_sample(s,phase=native.get('phase','IDLE'),
                target=native.get('sender_target_enu'),feedback=getattr(self,'target_feedback',None),
                observation=observation_status(self.settings,s),sources=sensor_sources(self.samples),owner='cpp_observer'))
        native = getattr(self,'last_native_status',{})
        airborne_start = self.settings.start_mode == 'takeoff_hold' and s.armed
        # Readiness exists before an operator command. Request age/persistence
        # is checked independently when the native executor accepts it.
        readiness = copy.copy(self.session);readiness.last_web_s = now
        reason = MissionChain.ready(readiness, s, now, ground=not airborne_start)
        if airborne_start and not native.get('native_takeoff',{}).get('can_start'):reason = 'position_hold_not_ready'
        if not s.local_setpoint_ready: reason = 'local_transport_unavailable'
        if type(s.rc_override) is not int or not s.rc_override & 2: reason = 'px4_offboard_rc_override_required'
        r = self.session.intent
        if r and self.native_session and (r['action'] != 'start' or reason == 'ready'):
            if r['request_id'] not in self.session.consumed:
                future = self.journal.submit(r['request_id'])
                durable = future.done() and future.exception() is None
                if r['action'] == 'land' or (durable and recording_ok):
                    # A requested LAND bypasses storage; ordinary admission
                    # still waits for the exact request to be persisted.
                    self.session.consumed.add(r['request_id'])
                    self.session.ack = {k:r[k] for k in ('action','request_id','requested_at')}
                    self.forwarded = dict(r, ceiling_height_m=self.session.cpp_ceiling,
                        executor_session=self.native_session, request_persisted=durable)
                    self.record(dict(type='request_forwarded', durable_before_forward=durable, request=self.forwarded))
                    if r['action'] == 'start': self.session.assignment = r
                    self.session.intent = None
                else:
                    self.session.validation = dict(accepted=False, reason='request_record_pending' if not future.done() else 'request_record_failed', request_id=r['request_id'])
        if self.forwarded:
            future = self.journal.requests[self.forwarded['request_id']]
            self.forwarded['request_persisted'] = future.done() and future.exception() is None
        state = asdict(s); state.pop('origin')
        state.update(web_age_s=now-self.session.last_web_s if self.session.last_web_s is not None else None,
                     ground_gate_ready=reason == 'ready', request_persisted=bool(self.forwarded and self.forwarded.get('request_persisted')),
                     max_up_m_s=self.up_limit,max_up_age_s=now-self.up_limit_received,
                     rc_calibration=getattr(self,'rc_calibration',{}))
        envelope = dict(schema='sangwon-cpp-observer/1', executor_session=self.native_session,
                        settings=asdict(self.settings), sample_time_s=now,
                        state=clean(state), request=self.forwarded,
                        preflight=field_preflight(self.settings,s,map_loaded=False,
                            recording_ok=recording_ok,cpp_ai_ready=self.native_session is not None))
        up_ok = type(self.up_limit) in (float,int) and 0 < self.up_limit <= .5 and now-self.up_limit_received <= 5
        envelope['preflight']['checks'].append(dict(code='native_up_limit',title='PX4 상승 제한 0.5m/s 이하',passed=up_ok,required=True))
        if airborne_start:
            for check in envelope['preflight']['checks']:
                if check['code'] == 'ground':
                    check.update(code='airborne_position_hold',title='POSITION 안정 확인·별도 START 대기',
                        passed=native.get('native_takeoff',{}).get('can_start') is True)
                # A delayed status report is diagnostic once airborne. The
                # native gate still requires real fresh XY/UWB and stability.
                if check['code'] in ('estimator','vertical_estimator','bridge'):
                    check['passed'] = {'estimator':s.estimator_valid,'vertical_estimator':s.vertical_estimator_valid,
                        'bridge':not observation_status(self.settings,s)['contract_error']}[check['code']]
        checks = envelope['preflight']['checks']
        envelope['preflight']['checked_inputs_passed'] = all(c['passed'] for c in checks if c['required'])
        envelope['preflight']['blockers'] = [c['code'] for c in checks if c['required'] and not c['passed']]
        envelope['storage'] = dict(request_journal=self.journal.worker.status(), evidence=self.recorder.status() if self.recorder else None)
        self.output.publish(String(data=json.dumps(envelope, allow_nan=False, separators=(',', ':'))))

    def destroy_node(self):
        self.journal.worker.close()
        if self.recorder: self.recorder.close()
        return super().destroy_node()


def main(args=None):
    from drone_uwb.integration.ros.lifecycle import init_for_main, spin_until_stopped
    init_for_main(args, interrupt_spin=False)
    node = CppObserver()
    try: spin_until_stopped(node)
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
