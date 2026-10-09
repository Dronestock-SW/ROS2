from unittest.mock import patch
import time
import pytest
from drone_platform_link.telemetry import Observations


@pytest.mark.parametrize('source',['uwb_xy','btf_xy'])
def test_xy_sources_never_export_pose_z_placeholder(source):
    o=Observations(source)
    assert o.receive_pose(1,2,'uwb_map',1_000_000_000,now_ns=1_100_000_000,z=0)
    f=o.fields()
    assert f['pose_source']==source and f['fix']
    assert f['current_z_m'] is None and f['z_source'] is None
    assert not f['xyz_valid']
    assert f['source_age_ms']>=100


def test_original_age_does_not_restart_at_receipt_or_duplicate():
    with patch('drone_platform_link.telemetry.time.monotonic',return_value=10):
        o=Observations('btf_xy')
        assert o.receive_pose(1,2,'uwb_map',1_000_000_000,now_ns=1_400_000_000)
        assert not o.receive_pose(9,9,'uwb_map',1_000_000_000,now_ns=1_400_000_000)
    with patch('drone_platform_link.telemetry.time.monotonic',return_value=10.15):
        f=o.fields()
    assert f['source_age_ms']==550
    assert not f['fix'] and f['x'] is None


def test_px4_source_retains_local_frame_and_same_stamp_xyz():
    o=Observations('px4_local')
    assert o.receive_pose(1,2,'map',1_000_000_000,now_ns=1_000_000_000,z=3)
    f=o.fields()
    assert f['coordinate_frame']=='px4_local_enu' and f['position_reference']=='fc'
    assert f['position_kind']=='state_estimate' and f['xyz_valid']
    assert f['current_z_m']==3 and f['source_stamp_ns']==1_000_000_000
    assert not o.receive_pose(1,2,'uwb_map',2_000_000_000,now_ns=2_000_000_000,z=3)
    assert not o.fields()['fix']


def test_btf_height_is_same_stamp_xyz_and_never_relabels_px4_height():
    stamp = time.time_ns()
    btf = Observations('btf_xy')
    btf.receive_pose(1., 2., 'uwb_map', stamp, now_ns=stamp)
    btf.receive_height(.7, 'uwb_map', stamp)
    assert btf.fields()['xyz_valid']
    btf.receive_height(.7, 'uwb_map', stamp-1_000_000)
    assert not btf.fields()['xyz_valid']
    px4 = Observations('px4_local')
    px4.receive_pose(1., 2., 'map', stamp, now_ns=stamp, z=3.)
    px4.receive_height(.7, 'uwb_map', stamp)
    assert px4.fields()['current_z_m'] == 3.
    assert px4.fields()['coordinate_frame'] == 'px4_local_enu'


def test_partial_scan_progress_reaches_web_and_expires_with_mission_state():
    progress=dict(scan_failure_policy='continue_remaining_tasks',attempted_task_ids=['P1','S1'],
                  remaining_task_ids=['P2','S2','P3'],failed_scan_task_ids=['S1'],route_complete=False,
                  preflight={'checked_inputs_passed':False,'blockers':['rc']},home_xy_m=[2.,2.],fc_landed=1)
    o=Observations('btf_xy')
    with patch('drone_platform_link.telemetry.time.monotonic',return_value=10.):
        o.receive_mission(dict(state='MOVING',**progress))
        assert all(o.fields()[k]==v for k,v in progress.items())
    with patch('drone_platform_link.telemetry.time.monotonic',return_value=10.6):
        assert 'remaining_task_ids' not in o.fields()
        assert 'preflight' not in o.fields()


@pytest.mark.parametrize('armed', [False, True])
def test_explicit_ground_reference_is_estimated_for_armed_and_disarmed(armed):
    o = Observations('btf_xy', .15)
    with patch('drone_platform_link.telemetry.time.monotonic', return_value=10.):
        o.receive_fc(True, armed, 'POSCTL')
        o.receive_mission(dict(state='IDLE', fc_landed=1))
        f = o.fields()
        assert f['current_z_m'] == .15 and f['current_z_estimated']
        assert f['current_z_source'] == 'ground_antenna_reference'
        assert not f['current_z_trusted'] and not f['xyz_valid'] and not f['fix']
    with patch('drone_platform_link.telemetry.time.monotonic', return_value=10.6):
        assert o.fields()['current_z_m'] is None


@pytest.mark.parametrize('landed', [None, 0, 2, 3, 4])
def test_no_fixed_height_in_air_or_unknown_ground_state(landed):
    o = Observations('btf_xy', .15)
    o.receive_fc(True, True, 'AUTO.TAKEOFF')
    o.receive_mission(dict(state='TAKING_OFF', fc_landed=landed))
    assert o.fields()['current_z_m'] is None


def test_real_height_wins_and_px4_height_is_never_replaced():
    for source in ('btf_xy', 'px4_local'):
        o = Observations(source, .15)
        o.receive_fc(True, True, 'POSCTL')
        o.receive_mission(dict(state='IDLE', fc_landed=1))
        now = time.time_ns()
        o.receive_height(.62, 'uwb_map', now)
        if source == 'px4_local':
            o.receive_pose(0., 0., 'map', now, now_ns=now, z=-.07)
        f = o.fields()
        assert f['current_z_m'] == (.62 if source=='btf_xy' else -.07)
        assert f['current_z_trusted'] and not f['current_z_estimated']
