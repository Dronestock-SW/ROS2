"""Four requested bench sequences as synthetic plans, without hardware authority."""
import math

ROUTES = {
    'hover': [(0, 0)],
    'x': [(0, 0), (1, 0), (0, 0)],
    'y': [(0, 0), (0, 1), (0, 0)],
    'xy': [(0, 0), (1, 0), (1, 1), (1, 0), (0, 0)],
}


def make_plan(case, height):
    if case not in ROUTES:
        raise ValueError('Unknown bench case')
    if type(height) not in (int, float) or not math.isfinite(height) or not .1 <= height <= 5:
        raise ValueError('Synthetic height must be finite, between 0.1 and 5 m')
    # Zero is a synthetic launch reference, never the current warehouse origin.
    tasks = [dict(task_id=f'BENCH-{i:02}', type='waypoint',
                  position_m=dict(x=x, y=y, z=float(height)),
                  hold_s=3.0 if i == 0 else .5, yaw_deg=0)
             for i, (x, y) in enumerate(ROUTES[case])]
    return dict(contract_version='1.1-draft.4', type='mission_snapshot', profile='REPLAY',
        drone_id='TEST-DRONE-01', snapshot_id=f'bench-{case}-z{height:g}',
        mission_db_id=907, mission_code='BENCH-SYNTHETIC-NO-AIRCRAFT', route_revision='r1',
        required_capabilities=['mission_xyz', 'arrival_yaw', 'command_session',
                               'readiness_binding', 'replay_waypoints_v1'],
        coordinate_frame=dict(id='WAREHOUSE_MAP', length_unit='m', z_reference='SYNTHETIC_FLAT_FLOOR'),
        start_mode='AUTO_TAKEOFF', takeoff_z_m=float(height), start_yaw_deg=0,
        route_tasks=tasks, validation_scope='ALLOWLISTED_SYNTHETIC_ROUTE_ONLY',
        motion_profile_ref=dict(id='COMPILED_REPLAY_SIM01', revision='v0.1.0'),
        physical_flight_approval=False)


def audit_view(record=None):
    record = record or {}
    params = record.get('parameters', {})
    rc = record.get('last_messages', {}).get('/mavros/rc/in', {})
    channels = rc.get('channels', [])
    return dict(scope='LAST_RECORDED_READBACK_NOT_LIVE', sampled_at=record.get('utc'),
        can_fly=False, physical_output_enabled=False, anchor_z_m=.15, restore_anchor_z_m=2.2,
        parameters={k: params.get(k) for k in ('EKF2_EV_CTRL', 'EKF2_HGT_REF', 'EKF2_RNG_CTRL',
            'COM_RC_OVERRIDE', 'COM_RC_STICK_OV', 'COM_OBL_RC_ACT', 'COM_OF_LOSS_T',
            'COM_FAIL_ACT_T', 'MIS_TAKEOFF_ALT', 'RC_MAP_FLTMODE')},
        valid_rc_channels=sum(type(v) is int and 800 <= v <= 2200 for v in channels),
        blockers=['실제 PX4 명령 writer 미완료', 'UWB 연속성·좌표/장착/시각 검증 미완료',
                  'RC 스틱·스위치 인계 미검증', '실제 SITL·EKF 융합·고도 원점 미검증'])
