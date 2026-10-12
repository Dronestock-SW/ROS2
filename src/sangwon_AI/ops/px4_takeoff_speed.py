#!/usr/bin/env python3
"""Read or explicitly set the PX4 upward and native takeoff speed limits.

Requires fresh, connected, disarmed, landed FC data before every request.
Never sends flight commands. XY, landing speed, hover thrust and EKF are unchanged.
"""
import argparse
import json
from pathlib import Path
import time

NAMES = ['MPC_Z_VEL_MAX_UP', 'MPC_TKO_SPEED', 'MPC_Z_V_AUTO_UP']


def targets(speed, native_takeoff=False):
    values = dict.fromkeys(NAMES, speed)
    if native_takeoff:
        if speed <= .3:
            raise ValueError('Native PX4 takeoff requires upward speed above 0.3 m/s')
        # Pinned PX4 Auto reActivate starts smoothing at +0.7 m/s. A lower
        # descent ceiling repeatedly triggers emergency braking before liftoff.
        values['MPC_Z_VEL_MAX_DN'] = .6
    return values


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--speed', type=float, choices=(.25, .5), default=.5)
    p.add_argument('--apply', action='store_true')
    p.add_argument('--native-takeoff', action='store_true')
    p.add_argument('--output', type=Path, required=True)
    a = p.parse_args()
    desired = targets(a.speed, a.native_takeoff)
    names = list(desired)
    import rclpy
    from rclpy.qos import qos_profile_sensor_data as qos
    from mavros_msgs.msg import State, ExtendedState
    from mavros_msgs.srv import ParamPull
    from rcl_interfaces.srv import GetParameters, SetParameters
    from rcl_interfaces.msg import Parameter, ParameterValue
    rclpy.init()
    n = rclpy.create_node('takeoff_speed_settings')
    state = {}
    def update(key, msg):
        stamp = msg.header.stamp.sec*1e9 + msg.header.stamp.nanosec
        state[key] = (msg, time.monotonic(), stamp)
    n.create_subscription(State, '/mavros/state', lambda m: update('fc', m), qos)
    n.create_subscription(ExtendedState, '/mavros/extended_state', lambda m: update('landed', m), qos)
    result = dict(flight_commands=False, requested_speed=a.speed, apply=a.apply,
                  parameters=desired, writes=[], verified=False)
    def guard():
        now = time.monotonic()
        for key in ('fc', 'landed'):
            # MAVROS extended state is normally published at 1 Hz.
            # Use the same 2.5 s freshness bound as the existing FC readback.
            if key not in state or now-state[key][1] > 2.5:
                raise RuntimeError('Fresh FC state and landing state required')
            age = (n.get_clock().now().nanoseconds-state[key][2])/1e9
            if not 0 <= age <= 2.5:
                raise RuntimeError('FC state timestamp stale')
        fc = state['fc'][0]
        if not fc.connected or fc.armed or state['landed'][0].landed_state != 1:
            raise RuntimeError('Connected DISARM and landed required')
    def call(service, name, request, timeout=20):
        client = n.create_client(service, name)
        try:
            if not client.wait_for_service(timeout_sec=3):
                raise RuntimeError('Service missing: '+name)
            guard()
            future = client.call_async(request)
            end = time.monotonic()+timeout
            while not future.done() and time.monotonic() < end:
                rclpy.spin_once(n, timeout_sec=.1)
                guard()
            if not future.done():
                raise RuntimeError('Request outcome unknown: '+name)
            return future.result()
        finally:
            n.destroy_client(client)
    def read():
        pull = call(ParamPull, '/mavros/param/pull', ParamPull.Request(force_pull=True), 45)
        if not pull.success:
            raise RuntimeError('Fresh FC parameter pull failed')
        response = call(GetParameters, '/mavros/param/get_parameters', GetParameters.Request(names=names))
        if len(response.values) != len(names) or any(v.type != 3 for v in response.values):
            raise RuntimeError('Expected floating-point speed parameters')
        return dict(zip(names, (v.double_value for v in response.values)))
    try:
        end = time.monotonic()+4
        while time.monotonic() < end:
            rclpy.spin_once(n, timeout_sec=.1)
        result['before'] = read()
        if a.apply:
            for name, value in desired.items():
                request = SetParameters.Request(parameters=[Parameter(
                    name=name, value=ParameterValue(type=3, double_value=value))])
                response = call(SetParameters, '/mavros/param/set_parameters', request)
                result['writes'].append(dict(name=name, successful=bool(response.results and response.results[0].successful)))
                if not response.results or not response.results[0].successful:
                    raise RuntimeError('Parameter write failed: '+name)
            result['after'] = read()
            result['verified'] = all(abs(v-desired[k])<1e-5 for k,v in result['after'].items())
            if not result['verified']:
                raise RuntimeError('FC speed readback mismatch')
        print(json.dumps(result))
    finally:
        a.output.parent.mkdir(parents=True, exist_ok=True)
        a.output.write_text(json.dumps(result, indent=2)+'\n', encoding='utf-8')
        n.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
