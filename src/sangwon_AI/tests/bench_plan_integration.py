"""Run all four requested paths through the existing C++ core and FakePx4."""
import hashlib
from web_service_integration import Rig, wait_for, encode
from sangwon_web.bench_plan import ROUTES, make_plan


for case in ROUTES:
    plan = make_plan(case, 1.0)
    raw = encode(plan)
    with Rig(snapshot=raw, overrides={
            'replay_takeoff_policy': 'PX4_AUTO_TAKEOFF',
            'approved_replay_takeoff_reference': 'WAREHOUSE_MAP_SYNTHETIC_ONLY',
            'approved_replay_snapshot_sha256': [hashlib.sha256(raw).hexdigest()]}) as rig:
        rig.connect()
        rig.prepare()
        command = rig.command()
        rig.core.call('command', command)
        result = wait_for(lambda: (r if (r := rig.core.call('command.get',
            dict(control_request_id=command['control_request_id']))).get('result_revision') == 2 else None), timeout=40)
        assert result['flight_outcome'] == 'SUCCEEDED', result
        assert result['visited'] == len(plan['route_tasks']), result
        status = rig.core.call('status')
        assert status['physical_output_enabled'] is False
        assert status['readiness']['flight_authority'] is False
        assert status['mode_control']['observed_mode'] == 'AUTO.LAND'
        assert status['mode_control']['hardware_transport_implemented'] is False
    print(f'PASS {case}: C++ native takeoff / fixed-Z route / return / land; FakePx4 only')
