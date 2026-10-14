"""Observation-only MAVLink ODOMETRY through the existing MAVROS router.

No serial connection, FC parameter writes, flight commands, or FC state
feedback into observations. Static alignment is calibrated outside this node.
"""
import argparse
from collections import Counter
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

from drone_uwb.integration.local_odometry import (
    LocalObservationSettings, PARAMETERS, guarded_horizontal_odometry, parameter_gate,
    SenderStartGate, fresh_bridge_guard)
from drone_uwb.integration.clock_readiness import ClockReadiness


def main(args=None):
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser()
    parser.add_argument('--config', required=True)
    parser.add_argument('--same-user-dds', action='store_true',
                        help='Use existing DDS settings when all nodes share one OS user')
    options = parser.parse_args(args)
    config = json.loads(Path(options.config).read_text(encoding='utf-8'))
    settings = LocalObservationSettings(**config['settings'])
    if Path('/proc/sys/kernel/random/boot_id').read_text().strip() != settings.host_boot_id:
        raise RuntimeError('host_reboot_requires_new_alignment_evidence')
    from .local_transport import configure_local_dds
    if not options.same_user_dds:
        configure_local_dds()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data as qos
    from geometry_msgs.msg import PoseWithCovarianceStamped
    from mavros_msgs.msg import State, ExtendedState, TimesyncStatus, Mavlink
    from rcl_interfaces.srv import GetParameters
    from std_msgs.msg import String
    from mavros.mavlink import convert_to_rosmsg
    from pymavlink import mavutil
    from .lifecycle import init_for_main, shutdown_requested

    class LocalBridge(Node):
        def __init__(self):
            super().__init__('uwb_local_odometry_bridge')
            if self.context.get_domain_id() != settings.ros_domain_id:
                raise RuntimeError('tag_domain_mismatch')
            self.output = self.create_publisher(Mavlink, '/uas1/mavlink_sink', qos)
            self.status = self.create_publisher(String, '/uwb/local_odometry_status', 10)
            self.encoder = mavutil.mavlink.MAVLink(None, srcSystem=255, srcComponent=196)
            self.clock = ClockReadiness()
            self.guard = fresh_bridge_guard()
            self.state = None
            self.landed = None
            self.landed_at = -float('inf')
            self.start_gate = SenderStartGate()
            self.state_at = self.param_at = self.btf_at = self.request_at = -float('inf')
            self.params = {}
            self.future = None
            self.client = self.create_client(GetParameters, '/mavros/param/get_parameters')
            self.last_stamp = None
            self.reset_counter = 0
            self.session = None
            self.btf = {}
            self.decision = {}
            self.decision_at = -float('inf')
            self.max_remote_ns = settings.fc_remote_ns_at_alignment
            self.frozen = None
            self.counts = Counter()
            self.last_reason = 'starting'
            self.last_fields = None
            self.last_send_at = self.last_report_at = -float('inf')
            self.max_send_gap_s = 0.
            self.create_subscription(PoseWithCovarianceStamped, '/uwb/btf_pose', self.pose, qos)
            self.create_subscription(State, '/mavros/state', self.receive_state, qos)
            self.create_subscription(ExtendedState, '/mavros/extended_state',
                                     self.receive_landed, qos)
            self.create_subscription(TimesyncStatus, '/mavros/timesync_status', self.sync, qos)
            self.create_subscription(String, '/uwb/btf_status', self.btf_status, qos)
            self.create_subscription(String, '/uwb/btf_decision', self.btf_decision, qos)
            self.create_subscription(Mavlink, '/uas1/mavlink_source', self.inspect_source, qos)
            self.create_timer(.2, self.monitor)

        def receive_landed(self, msg):
            stamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds-stamp)/1e9
            if 0 <= age <= .5:
                self.landed, self.landed_at = msg.landed_state, time.monotonic()

        def receive_state(self, msg):
            stamp = msg.header.stamp.sec*1_000_000_000 + msg.header.stamp.nanosec
            age = (self.get_clock().now().nanoseconds-stamp)/1e9
            if not 0 <= age <= 2.5:
                self.state = None
                return
            if self.state is None or msg.connected != self.state.connected:
                self.clock.reset()
                self.params = {}
                self.param_at = -float('inf')
            self.state, self.state_at = msg, time.monotonic()

        def sync(self, msg):
            if msg.remote_timestamp_ns < self.max_remote_ns-1_000_000_000:
                self.frozen = 'fc_clock_reset_requires_alignment_review'
                return
            self.max_remote_ns = max(self.max_remote_ns, msg.remote_timestamp_ns)
            self.clock.observe(now=time.monotonic(), remote_ns=msg.remote_timestamp_ns,
                offset_ns=msg.estimated_offset_ns, rtt_ms=msg.round_trip_time_ms)

        def btf_status(self, msg):
            try:
                row = json.loads(msg.data)
                session = row['observation_session']
            except (ValueError, KeyError):
                return
            if self.session is not None and session != self.session:
                if self.state is None or self.state.armed or time.monotonic()-self.state_at > 2.5:
                    self.frozen = 'btf_session_changed_requires_disarmed_review'
                else:
                    self.guard = fresh_bridge_guard()
                    self.reset_counter = (self.reset_counter+1) % 256
            self.session, self.btf, self.btf_at = session, row, time.monotonic()

        def inspect_source(self, msg):
            if (msg.msgid in (102, 104, 138, 331)
                    and (msg.sysid, msg.compid) not in ((1, 1), (255, 196))
                    and not (msg.msgid == 331 and (msg.sysid, msg.compid) == (255, 198))):
                self.frozen = 'another_external_observation_writer'

        def btf_decision(self, msg):
            try:
                self.decision = json.loads(msg.data)
                self.decision_at = time.monotonic()
            except ValueError:
                self.decision = {}

        def gate(self):
            if not self.start_gate.authorized:
                now = time.monotonic()
                if self.state is None or not self.start_gate.check(
                        connected=self.state.connected, armed=self.state.armed,
                        landed_state=self.landed, state_age_s=now-self.state_at,
                        landed_age_s=now-self.landed_at):
                    return 'sender_requires_disarmed_ground_start'
            now = time.monotonic()
            if self.frozen:
                return self.frozen
            if self.state is None or not self.state.connected or not 0 <= now-self.state_at <= 2.5:
                return 'fcu_disconnected_or_stale'
            if not 0 <= now-self.param_at <= 5.:
                return 'fcu_parameters_unavailable'
            reason = parameter_gate(settings, self.params)
            if reason != 'ready':
                return reason
            if not self.clock.ready(now):
                return 'timesync_unavailable_or_unstable'
            # Status is emitted at 1 Hz; measurement freshness stays 150 ms.
            if self.session is None or not 0 <= now-self.btf_at <= 1.5:
                return 'btf_status_stale'
            if (self.count_publishers('/uwb/btf_pose') != 1
                    or self.count_publishers('/mavros/state') != 1
                    or self.count_publishers('/uwb/local_odometry_status') != 1
                    or self.output.get_subscription_count() != 1):
                # Keep one router sink, including for guarded FC readback.
                return 'ambiguous_or_missing_transport'
            from drone_uwb.integration.lidar_velocity import RAW_WRITERS
            allowed = RAW_WRITERS
            if any(info.node_name not in allowed for info in
                    self.get_publishers_info_by_topic('/uas1/mavlink_sink')):
                return 'unrecognized_raw_route_writer'
            if (self.count_publishers('/mavros/vision_pose/pose_cov')
                    or self.count_publishers('/mavros/odometry/in')):
                return 'another_external_observation_publisher'
            return 'ready'

        def monitor(self):
            now = time.monotonic()
            if self.future is not None and now-self.request_at > 2.:
                self.client.remove_pending_request(self.future)
                self.future = None
            if (self.state is not None and self.state.connected and self.future is None
                    and now-self.request_at >= 1. and self.client.service_is_ready()):
                self.request_at = now
                self.future = self.client.call_async(GetParameters.Request(names=list(PARAMETERS)))
                self.future.add_done_callback(self.parameters_received)
            row = {'gate': self.gate(), 'last_reason': self.last_reason,
                'status_stamp_ns':self.get_clock().now().nanoseconds,
                'status_monotonic_ns':round(now*1e9),
                'last_input_stamp_ns':getattr(self,'last_input_stamp_ns',0),
                'last_input_received_monotonic_ns':getattr(self,'last_input_received_monotonic_ns',0),
                'last_send_monotonic_ns':round(self.last_send_at*1e9) if self.last_fields else None,
                'last_odometry_time_usec':self.last_fields['time_usec'] if self.last_fields else None,
                'jump_guard_enabled':False,
                'observation_recovery_wait_s':self.guard.config.recover_s,
                'counts': dict(self.counts), 'last_sample_stamp_ns': self.last_stamp,
                'last_send_age_s': now-self.last_send_at if self.last_fields else None,
                'max_send_gap_s': self.max_send_gap_s, 'parameters': self.params,
                'settings': asdict(settings), 'observation_session': self.session,
                'reset_counter': self.reset_counter,
                'last_position_ned_xy_m': None if self.last_fields is None else
                    [self.last_fields['x'], self.last_fields['y']],
                'timesync_ready': self.clock.ready(now), 'frozen': self.frozen,
                'position_reference': 'uwb_antenna', 'lever_arm_owner': 'PX4_EKF2',
                'z_observed': False, 'attitude_observed': False, 'velocity_observed': False,
                'geographic_origin_required': False, 'flight_commands_enabled': False,
                'timing_calibrated': False, 'dynamic_alignment_verified': False,
                'flight_verified': False}
            self.status.publish(String(data=json.dumps(row, ensure_ascii=False)))
            if now-self.last_report_at >= 5.:
                print(json.dumps(row, ensure_ascii=False), flush=True)
                self.last_report_at = now

        def parameters_received(self, future):
            if future is not self.future:
                return
            self.future = None
            if future.exception() is None:
                self.params = {name: val.integer_value if val.type == 2 else
                    val.double_value if val.type == 3 else None
                    for name, val in zip(PARAMETERS, future.result().values)}
                self.param_at = time.monotonic()

        def pose(self, msg):
            stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
            self.last_input_stamp_ns=stamp
            self.last_input_received_monotonic_ns=time.monotonic_ns()
            reason = self.gate()
            if reason == 'ready':
                try:
                    c = msg.pose.covariance
                    reason, fields = guarded_horizontal_odometry(settings, self.guard,
                        motion_eligible=(self.decision.get('height_source') == 'measured_tof_imu'
                            and self.decision.get('height_ready') is True
                            and time.monotonic()-self.decision_at <= .1
                            and abs(stamp-self.decision.get('stamp_ns', 0)) <= 100_000_000),
                        frame=msg.header.frame_id,
                        stamp_ns=stamp, now_ns=self.get_clock().now().nanoseconds,
                        last_stamp_ns=self.last_stamp, x=msg.pose.pose.position.x,
                        y=msg.pose.pose.position.y, covariance=[[c[0], c[1]], [c[6], c[7]]],
                        reset_counter=self.reset_counter)
                except (ValueError, ArithmeticError) as exc:
                    reason = str(exc)
            self.last_reason = reason
            if reason != 'ready':
                self.counts[reason] += 1
                return
            e = mavutil.mavlink
            packet = self.encoder.odometry_encode(fields['time_usec'],
                e.MAV_FRAME_LOCAL_NED, e.MAV_FRAME_BODY_FRD,
                fields['x'], fields['y'], fields['z'], fields['q'],
                fields['vx'], fields['vy'], fields['vz'], fields['rollspeed'],
                fields['pitchspeed'], fields['yawspeed'], fields['pose_covariance'],
                fields['velocity_covariance'], fields['reset_counter'], e.MAV_ESTIMATOR_TYPE_VISION, 0)
            packet.pack(self.encoder)
            self.encoder.seq = (self.encoder.seq+1) % 256
            self.output.publish(convert_to_rosmsg(packet))
            now = time.monotonic()
            if self.last_fields is not None:
                self.max_send_gap_s = max(self.max_send_gap_s, now-self.last_send_at)
            self.last_fields, self.last_stamp, self.last_send_at = fields, stamp, now
            self.counts['published'] += 1

    init_for_main([])
    node = LocalBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except RuntimeError:
        if not shutdown_requested():
            raise
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
