#!/usr/bin/env python3
"""Capture operator alignment and apply only the horizontal EV parameter set.

Connected, DISARMED and ON_GROUND are required for every parameter write.
Never arms, selects a flight mode, or writes height/attitude/GPS settings.
"""
import argparse
from dataclasses import asdict
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import sys
import time

from drone_uwb.integration.local_odometry import (
    LocalObservationSettings, APPLIED_PARAMETERS, PARAMETERS, alignment_yaw_deg)
from drone_uwb.integration.ros.local_transport import configure_local_dds


def main():
    sys.stdout.reconfigure(encoding='utf-8')
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--heading', choices=('+X', '-X', '+Y', '-Y'), required=True)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--restore-from')
    options = parser.parse_args()
    output = Path(options.output)
    if output.exists():
        raise RuntimeError('new_evidence_path_required')
    configure_local_dds()
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data as qos
    from sensor_msgs.msg import Imu
    from mavros_msgs.msg import State, ExtendedState, TimesyncStatus
    from rcl_interfaces.msg import Parameter, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters
    rclpy.init()
    node = Node('uwb_local_alignment_and_parameter_setup')
    state = {}
    yaw_samples = []
    def receive(name, msg):
        state[name] = (msg, time.monotonic())
    node.create_subscription(State, '/mavros/state', lambda msg: receive('state', msg), qos)
    node.create_subscription(ExtendedState, '/mavros/extended_state', lambda msg: receive('landed', msg), qos)
    node.create_subscription(TimesyncStatus, '/mavros/timesync_status', lambda msg: receive('sync', msg), qos)
    def imu(msg):
        q = msg.orientation
        values = (q.x, q.y, q.z, q.w)
        stamp = msg.header.stamp.sec*1_000_000_000+msg.header.stamp.nanosec
        age = (node.get_clock().now().nanoseconds-stamp)/1e9
        if all(math.isfinite(v) for v in values) and 0 <= age <= .1:
            yaw_samples.append(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z)))
    node.create_subscription(Imu, '/mavros/imu/data', imu, qos)
    get_client = node.create_client(GetParameters, '/mavros/param/get_parameters')
    set_client = node.create_client(SetParameters, '/mavros/param/set_parameters')
    def guard():
        now = time.monotonic()
        for name, timeout in (('state', 2.5), ('landed', .5), ('sync', .5)):
            if name not in state or not 0 <= now-state[name][1] <= timeout:
                raise RuntimeError('fresh_fc_evidence_required:' + name)
        if (not state['state'][0].connected or state['state'][0].armed
                or state['landed'][0].landed_state != ExtendedState.LANDED_STATE_ON_GROUND):
            raise RuntimeError('connected_disarmed_ground_required')
        if state['sync'][0].round_trip_time_ms > 20:
            raise RuntimeError('timesync_rtt_too_large')
    def call(client, request):
        if not client.wait_for_service(timeout_sec=3.):
            raise RuntimeError('parameter_service_unavailable')
        if client is set_client:
            guard()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.)
        if not future.done() or future.exception() is not None:
            raise RuntimeError('parameter_service_failed')
        return future.result()
    def get_params():
        result = call(get_client, GetParameters.Request(names=list(PARAMETERS)))
        return {name: value.integer_value if value.type == 2 else value.double_value
            if value.type == 3 else None for name, value in zip(PARAMETERS, result.values)}
    evidence = {'checked_at_utc': datetime.now(timezone.utc).isoformat(),
        'operator_heading': options.heading, 'writes': [], 'flight_commands': False}
    def save():
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(evidence, indent=2, ensure_ascii=False), encoding='utf-8')
    try:
        start = time.monotonic()
        while time.monotonic()-start < 7.:
            rclpy.spin_once(node, timeout_sec=.05)
        guard()
        if len(yaw_samples) < 300:
            raise RuntimeError('stationary_heading_sample_count_insufficient')
        mean = math.atan2(sum(map(math.sin, yaw_samples)), sum(map(math.cos, yaw_samples)))
        deviations = [math.remainder(yaw-mean, 2*math.pi) for yaw in yaw_samples]
        span = math.degrees(max(deviations)-min(deviations))
        if span > 3.:
            raise RuntimeError('stationary_heading_span_exceeds_3_deg')
        body_map = {'+X': 0., '-X': 180., '+Y': 90., '-Y': -90.}[options.heading]
        settings = LocalObservationSettings(enabled=True,
            enu_yaw_deg=alignment_yaw_deg(math.degrees(mean), body_map, -1),
            host_boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
            fc_remote_ns_at_alignment=state['sync'][0].remote_timestamp_ns,
            alignment_method='operator_cardinal_and_stationary_px4_yaw',
            layout_confirmed=True, sensor_mount_confirmed=True)
        evidence.update(settings=asdict(settings), yaw_samples=len(yaw_samples),
            body_enu_yaw_deg=math.degrees(mean), yaw_span_deg=span,
            before=get_params(), mount_evidence='20261009 measured FC-to-tag FRD [-0.10,0,-0.10] m',
            layout_evidence='operator confirmed 6.3 x 4.6 m / anchor z=0.15 m on 20261011',
            timing_calibrated=False, dynamic_alignment_verified=False, flight_verified=False)
        save()  # Durable rollback evidence BEFORE any FC write.
        wanted = APPLIED_PARAMETERS
        if options.restore_from:
            old = json.loads(Path(options.restore_from).read_text(encoding='utf-8'))['before']
            wanted = {name: old[name] for name in APPLIED_PARAMETERS}
        if options.apply:
            for name, value in wanted.items():
                guard()
                parameter = Parameter(name=name, value=ParameterValue(
                    type=2, integer_value=value) if type(value) is int else
                    ParameterValue(type=3, double_value=float(value)))
                response = call(set_client, SetParameters.Request(parameters=[parameter]))
                evidence['writes'].append({'name': name, 'value': value,
                    'successful': bool(response.results and response.results[0].successful),
                    'reason': response.results[0].reason if response.results else 'empty_result'})
                save()
                if not evidence['writes'][-1]['successful']:
                    raise RuntimeError('parameter_write_rejected:' + name)
            evidence['after'] = get_params()
            for name, wanted_value in wanted.items():
                actual = evidence['after'][name]
                if actual is None or abs(actual-wanted_value) > .001:
                    raise RuntimeError('parameter_readback_mismatch:' + name)
        print(json.dumps(evidence, ensure_ascii=False))
    except Exception as exc:
        evidence['error'] = str(exc)
        save()
        raise
    finally:
        save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
