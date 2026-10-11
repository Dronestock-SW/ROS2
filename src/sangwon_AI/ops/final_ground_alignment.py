#!/usr/bin/env python3
"""Record a new stationary -Y heading alignment; no FC commands or parameters.

Operator must physically confirm the nose is pointing along anchor -Y.
Writes new private files only; never overwrites existing evidence.
"""
import argparse
import json
import math
from pathlib import Path
import time


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--confirm-nose-minus-y', action='store_true', required=True)
    for key in ('alignment-template', 'mission-template', 'output-dir'):
        p.add_argument('--' + key, type=Path, required=True)
    a = p.parse_args()
    if a.output_dir.exists():
        p.error('Use a new private output directory')
    import rclpy
    from geometry_msgs.msg import PoseStamped, TwistStamped
    from mavros_msgs.msg import State, ExtendedState, TimesyncStatus
    from rclpy.qos import qos_profile_sensor_data as qos
    rclpy.init()
    n = rclpy.create_node('final_ground_alignment')
    values, stamps, yaws = {}, {}, []
    def save(key, value):
        values[key], stamps[key] = value, time.monotonic()
    def pose(m):
        q = m.pose.orientation
        yaws.append(math.degrees(math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))))
        save('pose', True)
    n.create_subscription(PoseStamped, '/mavros/local_position/pose', pose, qos)
    n.create_subscription(State, '/mavros/state', lambda m: save('state', m.connected and not m.armed), qos)
    n.create_subscription(ExtendedState, '/mavros/extended_state', lambda m: save('landed', m.landed_state == 1), qos)
    n.create_subscription(TimesyncStatus, '/mavros/timesync_status', lambda m: save('remote', m.remote_timestamp_ns), qos)
    n.create_subscription(TwistStamped, '/mavros/local_position/velocity_local',
        lambda m: save('speed', math.sqrt(m.twist.linear.x**2+m.twist.linear.y**2+m.twist.linear.z**2)), qos)
    start = time.monotonic()
    try:
        while time.monotonic()-start < 8:
            rclpy.spin_once(n, timeout_sec=.1)
            if values.get('state') is False or values.get('landed') is False:
                raise RuntimeError('Connected DISARM and landed are required throughout')
            if values.get('speed', 0) > .1:
                raise RuntimeError('Stationary estimate required')
        now = time.monotonic()
        if any(now-stamps.get(k, -1e9) > 1 for k in ('state','landed','remote','pose','speed')):
            raise RuntimeError('Fresh FC observations required')
        if len(yaws) < 50 or max(yaws)-min(yaws) > 1:
            raise RuntimeError('Stable yaw samples required; wrap or movement detected')
        yaw = sum(yaws)/len(yaws)-90
        boot = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
        alignment = json.loads(a.alignment_template.read_text(encoding='utf-8'))
        mission = json.loads(a.mission_template.read_text(encoding='utf-8'))
        alignment['settings'].update(host_boot_id=boot, fc_remote_ns_at_alignment=values['remote'], enu_yaw_deg=yaw)
        alignment.update(operator_heading='-Y', yaw_samples=len(yaws), yaw_span_deg=max(yaws)-min(yaws),
                         flight_commands=False, flight_verified=False)
        mission.update(bridge_host_boot_id=boot, enu_yaw_deg=yaw)
        for key in ('map_y_axis_sign','enu_offset_x_m','enu_offset_y_m'):
            if mission[key] != alignment['settings'][key]:
                raise RuntimeError('Template coordinate conventions differ')
        a.output_dir.mkdir(parents=True)
        for name, data in [('alignment.json',alignment),('mission.json',mission)]:
            (a.output_dir/name).write_text(json.dumps(data, indent=2)+'\n', encoding='utf-8')
        print('Ground alignment saved. Dynamic accuracy and flight stability remain unverified.')
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
