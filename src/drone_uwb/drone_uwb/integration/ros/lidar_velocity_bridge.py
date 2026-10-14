"""One mode-independent measured LiDAR sender through the existing MAVROS.

No serial ownership, ARM, mode/setpoint command, or FC parameter write. A new
process requires DISARM ground evidence; a running sender survives RC takeover.
"""
import argparse
from collections import Counter, deque
from dataclasses import asdict
import json
import math
import os
from pathlib import Path
import time

import numpy as np
from scipy.spatial.transform import Rotation

from drone_uwb.processing.lidar_motion import LidarMotionSettings, MeasuredLidarMotion
from drone_uwb.integration.clock_readiness import ClockReadiness
from drone_uwb.integration.local_odometry import SenderStartGate
from drone_uwb.integration.lidar_velocity import (NODE_NAME, SOURCE_SYSTEM,
    SOURCE_COMPONENT, RAW_WRITERS, PARAMETERS, parameter_gate, velocity_fields, VelocityReadinessWindow)


def main(args=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--config', type=Path, required=True)
    p.add_argument('--record-directory', type=Path)
    p.add_argument('--same-user-dds', action='store_true')
    p.add_argument('--seconds', type=float, default=0.)
    options, ros_args = p.parse_known_args(args)
    settings = LidarMotionSettings(**json.loads(options.config.read_text(encoding='utf-8'))['settings'])
    os.environ['MAVLINK20'] = '1'
    from .local_transport import configure_local_dds
    if not options.same_user_dds:
        configure_local_dds()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data as qos
    from rcl_interfaces.srv import GetParameters
    from sensor_msgs.msg import Imu, PointCloud, Range
    from mavros_msgs.msg import State, ExtendedState, TimesyncStatus, Mavlink
    from std_msgs.msg import String
    from mavros.mavlink import convert_to_rosmsg
    from pymavlink import mavutil

    class LidarBridge(Node):
        def __init__(self):
            super().__init__(NODE_NAME)
            if self.context.get_domain_id() != (1 if settings.tag_id == '5' else 2):
                raise RuntimeError('lidar_tag_domain_mismatch')
            self.motion = MeasuredLidarMotion(settings)
            self.clock = ClockReadiness()
            self.start_gate = SenderStartGate()
            self.state = self.landed = self.floor = None
            self.state_at = self.landed_at = self.param_at = self.request_at = -math.inf
            self.params = {}
            self.pending = None
            self.last_sent_ns = None
            self.last_send_at = None
            self.last_imu_ns = None
            self.last_ros_ns = None
            self.frozen = None
            self.queue = deque(maxlen=2)
            self.counts = Counter()
            self.last = {}
            self.last_published = {}
            self.readiness = VelocityReadinessWindow()
            self.started = time.monotonic()
            self.output = self.create_publisher(Mavlink, '/uas1/mavlink_sink', qos)
            self.status = self.create_publisher(String, '/lidar/velocity_status', 10)
            self.observation = self.create_publisher(String, '/lidar/velocity_observation', qos)
            self.encoder = mavutil.mavlink.MAVLink(None, srcSystem=SOURCE_SYSTEM, srcComponent=SOURCE_COMPONENT)
            self.client = self.create_client(GetParameters, '/mavros/param/get_parameters')
            self.file = None
            if options.record_directory:
                options.record_directory.mkdir(parents=True, exist_ok=False)
                self.file = (options.record_directory/'observations.jsonl').open('x',encoding='utf-8')
                (options.record_directory/'settings.json').write_text(
                    json.dumps(asdict(settings),indent=2)+'\n',encoding='utf-8')
            self.create_subscription(PointCloud, '/point_cloud', self.cloud, qos)
            self.create_subscription(Imu, '/mavros/imu/data', self.imu, qos)
            self.create_subscription(Range, '/mavros/downward_0', self.height, qos)
            self.create_subscription(State, '/mavros/state', self.receive_state, qos)
            self.create_subscription(ExtendedState, '/mavros/extended_state', self.receive_landed, qos)
            self.create_subscription(TimesyncStatus, '/mavros/timesync_status', self.sync, qos)
            self.create_timer(.02, self.process)
            self.create_timer(.1, self.monitor)

        @staticmethod
        def stamp(msg):
            return msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec

        def fresh(self, msg, limit):
            return 0 <= (self.get_clock().now().nanoseconds-self.stamp(msg))/1e9 <= limit

        def imu(self, msg):
            ns = self.stamp(msg)
            if (msg.header.frame_id != settings.imu_frame or not self.fresh(msg,.2)
                    or msg.orientation_covariance[0] < 0 or msg.angular_velocity_covariance[0] < 0):
                self.counts['imu_frame_or_validity'] += 1
                return
            q = msg.orientation
            a = msg.angular_velocity
            try:
                raw = np.array([q.x,q.y,q.z,q.w])
                if abs(np.linalg.norm(raw)-1.) > .01:
                    raise ValueError('imu_quaternion_invalid')
                self.motion.imu.add_gyro(ns/1e9, [a.x,a.y,a.z])
                self.motion.imu.add_attitude(ns/1e9, Rotation.from_quat(raw).as_matrix())
                self.last_imu_ns = ns
                self.counts['imu'] += 1
            except ValueError:
                self.counts['imu_invalid'] += 1

        def height(self, msg):
            if self.fresh(msg,.15) and msg.min_range <= msg.range <= msg.max_range:
                self.floor = (msg.range, self.stamp(msg))
            else:
                self.floor = None

        def receive_state(self, msg):
            if not self.fresh(msg,2.5):
                return
            if self.state is None or self.state.connected != msg.connected:
                self.clock.reset()
                self.reset_motion()
                self.params.clear()
            self.state, self.state_at = msg, time.monotonic()

        def reset_motion(self):
            self.motion.reset_clock()
            self.readiness = VelocityReadinessWindow()
            self.last_published = {}

        def receive_landed(self, msg):
            if self.fresh(msg,.5):
                self.landed, self.landed_at = msg.landed_state, time.monotonic()

        def sync(self, msg):
            if self.clock.remote_ns and msg.remote_timestamp_ns < self.clock.remote_ns-1_000_000_000:
                self.frozen = 'fc_reboot_restart_disarmed_required'
                self.reset_motion()
                self.queue.clear()
                return
            _, changed = self.clock.observe(now=time.monotonic(), remote_ns=msg.remote_timestamp_ns,
                offset_ns=msg.estimated_offset_ns, rtt_ms=msg.round_trip_time_ms)
            if changed:
                self.reset_motion()
                self.queue.clear()
                self.counts['clock_step'] += 1

        def cloud(self, msg):
            self.counts['cloud'] += 1
            if len(self.queue) == self.queue.maxlen:
                self.queue.popleft()
                self.counts['queue_dropped'] += 1
                self.motion.break_continuity()
            self.queue.append((msg,time.monotonic()))

        def gate(self):
            if self.frozen:
                return self.frozen
            now = time.monotonic()
            if self.state is None or not self.start_gate.check(connected=self.state.connected,
                    armed=self.state.armed, landed_state=self.landed,
                    state_age_s=now-self.state_at, landed_age_s=now-self.landed_at):
                return 'sender_requires_disarmed_ground_start'
            if not self.state.connected or now-self.state_at > 2.5:
                return 'fcu_disconnected_or_stale'
            reason = parameter_gate(settings,self.params)
            if reason != 'ready':
                return reason
            if now-self.param_at > 5.:
                return 'fcu_parameters_stale'
            if not self.clock.ready(now):
                return 'timesync_unavailable_or_unstable'
            for topic in ('/point_cloud','/mavros/imu/data','/mavros/state','/lidar/velocity_status'):
                if self.count_publishers(topic) != 1:
                    return 'ambiguous_or_missing_source:'+topic
            if self.output.get_subscription_count() != 1:
                return 'ambiguous_or_missing_router'
            if any(i.node_name not in RAW_WRITERS for i in self.get_publishers_info_by_topic('/uas1/mavlink_sink')):
                return 'unrecognized_raw_route_writer'
            return 'ready'

        def process(self):
            ns = self.get_clock().now().nanoseconds
            if self.last_ros_ns is not None and (ns < self.last_ros_ns or ns-self.last_ros_ns > 1_000_000_000):
                self.reset_motion()
                self.clock.reset()
                self.queue.clear()
                self.counts['ros_clock_or_processing_gap'] += 1
            self.last_ros_ns = ns
            if not self.queue:
                return
            msg, received = self.queue[0]
            channels = [c.values for c in msg.channels if c.name == 'stamps']
            # Wait briefly for the final IMU bracket; keep original capture times.
            last_ns = self.stamp(msg)+round((channels[0][-1] if channels and channels[0] else 0.)*1e9)
            if (settings.geometry_gate() == 'ready' and time.monotonic()-received < .05
                    and (self.last_imu_ns is None or self.last_imu_ns < last_ns)):
                return
            self.queue.popleft()
            begin = time.monotonic()
            if len(channels) != 1:
                row = dict(accepted=False,reasons=['sdk_stamps_channel_missing'])
            elif not self.clock.ready(begin):
                self.motion.break_continuity()
                row = dict(accepted=False,reasons=['timesync_unavailable_or_unstable'])
            else:
                floor, stamp = self.floor or (math.nan, 0)
                row = self.motion.observe(points=[[p.x,p.y,p.z] for p in msg.points], offsets=list(channels[0]),
                    first_stamp_ns=self.stamp(msg), now_ns=ns, receipt_age_s=begin-received,
                    frame_id=msg.header.frame_id, floor_range_m=floor, floor_stamp_ns=stamp)
            row.update(processing_s=time.monotonic()-begin, received_age_s=begin-received,
                recorded_ros_ns=ns, gate=self.gate(), mode=None if self.state is None else self.state.mode,
                published=False, fusion_verified=False)
            if row['accepted'] and row['gate'] == 'ready':
                try:
                    fields = velocity_fields(row, now_ns=self.get_clock().now().nanoseconds,
                        last_stamp_ns=self.last_sent_ns, max_age_s=settings.max_observation_age_s)
                    e = mavutil.mavlink
                    packet = self.encoder.odometry_encode(fields['time_usec'], e.MAV_FRAME_LOCAL_NED,
                        e.MAV_FRAME_LOCAL_NED, fields['x'], fields['y'], fields['z'], fields['q'],
                        fields['vx'], fields['vy'], fields['vz'], fields['rollspeed'],fields['pitchspeed'],
                        fields['yawspeed'], fields['pose_covariance'], fields['velocity_covariance'],
                        fields['reset_counter'], e.MAV_ESTIMATOR_TYPE_LIDAR, fields['quality'])
                    packet.pack(self.encoder)
                    self.encoder.seq = (self.encoder.seq+1)%256
                    self.output.publish(convert_to_rosmsg(packet))
                    self.last_sent_ns, self.last_send_at = row['representative_stamp_ns'], time.monotonic()
                    self.counts['published'] += 1
                    row['published'] = True
                    self.last_published = dict(row)
                    self.readiness.observe(self.last_published)
                except ValueError as error:
                    row['gate'] = str(error)
            reason = 'accepted' if row['accepted'] else row['reasons'][0]
            self.counts[reason] += 1
            self.last = row
            text = json.dumps(row,ensure_ascii=False,allow_nan=False)
            self.observation.publish(String(data=text))
            if self.file:
                self.file.write(text+'\n')
                self.file.flush()

        def monitor(self):
            now = time.monotonic()
            if self.pending is not None and self.pending.done():
                if self.pending.exception() is None:
                    self.params = {n:v.integer_value if v.type==2 else v.double_value if v.type==3 else None
                        for n,v in zip(PARAMETERS,self.pending.result().values)}
                    self.param_at = now
                self.pending = None
            if self.pending is not None and now-self.request_at > 2.:
                self.client.remove_pending_request(self.pending)
                self.pending = None
            if self.pending is None and now-self.request_at >= 1. and self.client.service_is_ready():
                self.pending = self.client.call_async(GetParameters.Request(names=list(PARAMETERS)))
                self.request_at = now
            row = dict(gate=self.gate(), geometry_gate=settings.geometry_gate(), counts=dict(self.counts),
                status_stamp_ns=self.get_clock().now().nanoseconds,
                last_sent_stamp_ns=self.last_sent_ns,
                last_published_observation=self.last_published,
                stationary_window=self.readiness.status(),
                parameters=self.params, settings=asdict(settings), last_observation=self.last,
                last_send_age_s=None if self.last_send_at is None else now-self.last_send_at,
                mode=None if self.state is None else self.state.mode,
                mode_independent=True, z_observed=False, flight_commands_enabled=False,
                parameter_writes_enabled=False, fusion_verified=False, physical_accuracy_verified=False)
            self.status.publish(String(data=json.dumps(row,ensure_ascii=False,allow_nan=False)))

    rclpy.init(args=ros_args)
    node = LidarBridge()
    try:
        while rclpy.ok() and (not options.seconds or time.monotonic()-node.started < options.seconds):
            rclpy.spin_once(node,timeout_sec=.05)
    except KeyboardInterrupt:
        pass
    finally:
        if options.record_directory:
            (options.record_directory/'summary.json').write_text(json.dumps(dict(
                counts=dict(node.counts),gate=node.gate(),parameters=node.params,last=node.last),
                ensure_ascii=False,indent=2,allow_nan=False)+'\n',encoding='utf-8')
        if node.file:
            node.file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
