import hashlib
import json
from types import SimpleNamespace
import pytest
from drone_mission.native_ai import NativePlanner
from drone_mission.session import native_estimator_valid
from drone_mission.contracts import Settings


@pytest.mark.parametrize('constant,aiding,valid', [(True,True,True),(True,False,False),(False,True,True)])
def test_stationary_takeoff_vs_fake_position(constant,aiding,valid):
    m=SimpleNamespace(attitude_status_flag=True,velocity_horiz_status_flag=True,
        pos_horiz_rel_status_flag=True,const_pos_mode_status_flag=constant,
        pred_pos_horiz_rel_status_flag=aiding,gps_glitch_status_flag=False,accel_error_status_flag=False)
    assert native_estimator_valid(m) is valid
    m.accel_error_status_flag=True
    assert not native_estimator_valid(m)
    m.accel_error_status_flag=False;m.pos_horiz_rel_status_flag=False
    assert not native_estimator_valid(m)


def test_map_pin_and_poll_ack_do_not_recompile_or_change_command(tmp_path,monkeypatch):
    path=tmp_path/'map.json';path.write_bytes(b'{"survey_status":"SURVEYED"}');sha=hashlib.sha256(path.read_bytes()).hexdigest()
    with pytest.raises(ValueError,match='hash_mismatch'):NativePlanner('bin',path,'0'*64)
    p=NativePlanner('bin',path,sha);calls=[]
    route=[dict(id='S',type='scan',x=3,y=2,staging_xy_m=[2.5,2],path_validation_ref='placeholder')]
    def run(*args,**kwargs):
        calls.append(kwargs)
        return SimpleNamespace(returncode=0,stdout=json.dumps(dict(schema='sangwon-native-plan/1',
            validated=True,flight_authority=False,scope='SINGLE_ALTITUDE_STATIC_MAP',plan_ref='verified',
            route_tasks=[dict(route[0],path_validation_ref='verified')])))
    monkeypatch.setattr('drone_mission.native_ai.subprocess.run',run)
    payload=dict(route_tasks=route,control_action='start',control_request_id='start1',anchor_layout_id='layout')
    compiled,proof=p.compile(payload,(2,2),Settings())
    assert compiled['route_tasks'][0]['path_validation_ref']=='verified'
    ack=dict(payload,control_action=None,generated_at='later')
    result,_=p.reuse(ack)
    assert result['control_action'] is None and result['control_request_id']=='start1'
    land=dict(payload,control_action='land',control_request_id='land2')
    assert p.reuse(land)[0]['control_request_id']=='land2'
    assert len(calls)==1 and proof['map_sha256']==sha
    with pytest.raises(ValueError,match='not_prepared'):p.reuse(dict(payload,anchor_layout_id='changed'))


def test_compiler_cannot_change_route(tmp_path,monkeypatch):
    path=tmp_path/'map';path.write_bytes(b'{"survey_status":"SURVEYED"}')
    p=NativePlanner('bin',path,hashlib.sha256(path.read_bytes()).hexdigest())
    monkeypatch.setattr('drone_mission.native_ai.subprocess.run',lambda *a,**k:SimpleNamespace(returncode=0,
        stdout=json.dumps(dict(schema='sangwon-native-plan/1',validated=True,flight_authority=False,
                              route_tasks=[dict(id='P',x=4,y=2)]))))
    with pytest.raises(ValueError,match='changed_mission'):
        p.compile(dict(route_tasks=[dict(id='P',x=2.4,y=2)]),(2,2),Settings())
