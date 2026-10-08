"""Exact-hash synthetic web bundle -> C++ planner/runtime -> durable result."""
import copy
import hashlib
import json
import time

from replay_scan_fixture import bundle
from web_service_integration import Rig
from sangwon_web.ipc import CoreError


def fixture():
    raw, config = bundle()
    data = json.loads(raw)
    data['required_capabilities'].append('replay_static_detour_v1')
    data['policy']['global_detour_enabled'] = True
    data['route_tasks'] = [
        {'task_id': 'D01', 'type': 'waypoint', 'position_m': {'x': 4.5, 'y': 1.5, 'z': 1}, 'hold_s': 0, 'yaw_deg': 90},
        {'task_id': 'D02', 'type': 'waypoint', 'position_m': {'x': 4.5, 'y': 3.5, 'z': 1.2}, 'hold_s': 0},
    ]
    boundary = copy.deepcopy(data['map']['altitude_zones'][0])
    boundary['id'] = 'synthetic-detour-boundary'
    data['map']['flight_boundary'] = {'status': 'PROVIDED', 'volumes': [boundary]}
    data['map']['no_fly_zones'] = {'status': 'PROVIDED', 'volumes': []}
    data['map']['ceilings'] = []
    data['map']['obstacles'] = [{'id': 'synthetic-detour-wall', 'revision': 'r1',
        'polygon_xy_m': [[2.3, 0], [3, 0], [3, 2.8], [2.3, 2.8]], 'z_min_m': 0, 'z_max_m': 3}]
    config['approved_replay_planner'] = {'resolution_m': .25, 'max_nodes': 8192,
        'max_expansions': 4096, 'max_length_m': 50, 'budget_ms': 50}
    return pack(data, config)


def pack(data, config):
    raw = json.dumps(data, sort_keys=True, separators=(',', ':')).encode()
    config = copy.deepcopy(config)
    config['approved_replay_snapshot_sha256'] = [hashlib.sha256(raw).hexdigest()]
    return raw, config


def run_detour():
    raw, config = fixture()
    with Rig(steps=20, snapshot=raw, overrides=config) as rig:
        rig.connect(); ready=rig.prepare();assert ready['can_start']
        assert ready['validation_scope']=='ALLOWLISTED_SYNTHETIC_MAP_STATIC_DETOUR_ONLY'
        assert rig.core.call('status')['navigation']['global_detour_enabled']
        start = rig.command(); assert rig.core.call('command', start)['status'] == 'ACCEPTED'
        end = time.monotonic() + 30
        peak_y = 0
        failure_events = set()
        while time.monotonic() < end:
            rig.core.call('link.update', {'connected': True, 'code': 'OK', 'observed_contract': '1.1-draft.4'})
            status = rig.core.call('status')
            failure_events.update(e for e in status['navigation']['recent_events']
                                  if 'GOAL_FAILURE' in e or 'ROUTE_DEADLINE' in e)
            peak_y = max(peak_y, status['telemetry']['pose_fused']['position_m']['y'])
            result = rig.core.call('command.get', {'control_request_id': start['control_request_id']})
            if result.get('result_revision') == 2: break
            time.sleep(.05)
        else: raise AssertionError('Planner execution did not finish')
        assert result['flight_outcome'] == 'SUCCEEDED' and result['visited'] == 2, {
            'outcome': result['flight_outcome'], 'visited': result['visited'], 'peak_y': peak_y,
            'phase': status['telemetry']['phase'], 'reason': status['telemetry']['reason'],
            'routes': status['navigation']['route_revision'], 'failures': sorted(failure_events),
            'events': status['navigation']['recent_events']}
        assert peak_y > 3.5 and status['navigation']['route_revision'] > 1
        rows = rig.pending_reports()
        routes = [r['body'] for r in rows if r['body'].get('event_type') == 'ROUTE_PLANNED']
        assert routes and all(r['details']['scope'] == 'SYNTHETIC_STATIC_MAP_ONLY' for r in routes)
        report = next(r['body'] for r in rows if r['body'].get('type') == 'execution_result')
        assert report['task_summary'] == {'total': 2, 'succeeded': 2, 'failed': 0, 'not_attempted': 0}
        assert not [r for r in rows if r['route'] == 'scan-task-results']


def reject_before_takeoff():
    raw, config = fixture()
    data = json.loads(raw)
    changes = [
        ('DETOUR_BOUNDARY_REQUIRED', lambda s: s['map']['flight_boundary'].update(status='NOT_PROVIDED', volumes=None)),
        ('DETOUR_BOUNDARY_REQUIRED', lambda s: s['map']['no_fly_zones'].update(status='NOT_PROVIDED', volumes=None)),
        ('DETOUR_CAPABILITY_REQUIRED', lambda s: s['required_capabilities'].remove('replay_static_detour_v1')),
        ('PLAN_GOAL_INVALID', lambda s: s['route_tasks'][0]['position_m'].update(x=2.5, y=1.5)),
    ]
    cases = []
    for code, change in changes:
        altered = copy.deepcopy(data);change(altered)
        body, settings = pack(altered, config);cases.append((body, settings, code))
    missing = copy.deepcopy(config);missing.pop('approved_replay_planner')
    cases.append((raw, missing, 'PLANNER_LIMITS_REQUIRED'))
    for body, settings, code in cases:
        with Rig(snapshot=body, overrides=settings) as rig:
            rig.connect()
            try: rig.prepare()
            except CoreError as error: assert code in str(error), (code, str(error))
            else: raise AssertionError('Unapproved detour was accepted')
            state = rig.core.call('status')
            assert not state['readiness']['can_start'] and state['telemetry']['px4']['armed'] is False


if __name__ == '__main__':
    run_detour(); reject_before_takeoff()
    print('PASS static-map detour, ordered tasks, return/land, durable route events, preparation rejection')
