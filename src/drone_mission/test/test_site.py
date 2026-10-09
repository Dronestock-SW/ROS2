import copy
import json
import time
import pytest
from drone_mission.contracts import Settings
from drone_mission.local_web import LocalPlatform
from drone_mission.site import ceiling_map
from drone_mission.session import route_fingerprint


def test_site_save_reload_and_start_snapshot(tmp_path):
    path=tmp_path/'site.json'
    settings=Settings(full_mission=True)
    p=LocalPlatform(settings=settings,site_file=path)
    with pytest.raises(ValueError):p.command('start',[dict(id='P',x=2,y=2)])
    result=p.command('set_ceiling',ceiling=3)
    assert result['flight_command_sent'] is False
    assert p.assignment['control_action'] is None
    p=LocalPlatform(settings=settings,site_file=path)
    first=p.command('start',[dict(id='P',x=2,y=2)])
    assert first['ceiling_height_m']==3
    with pytest.raises(ValueError):p.command('set_ceiling',ceiling=4)
    assert route_fingerprint(first)!=route_fingerprint(dict(first,ceiling_height_m=4))
    assert json.loads(path.read_text())['ceiling_height_m']==3
    with pytest.raises(ValueError):LocalPlatform(drone_id='6',site_file=path)


@pytest.mark.parametrize('value',[True,None,'3',float('nan'),float('inf'),0,-1,101])
def test_invalid_ceiling_does_not_issue_command_or_change_site(value):
    p=LocalPlatform()
    with pytest.raises(ValueError):p.command('set_ceiling',ceiling=value)
    assert p.site['ceiling_height_m'] is None and p.assignment['control_action'] is None


@pytest.mark.parametrize('state,armed,landed,age',[
    ('MOVING',True,2,0),('END',False,1,0),('IDLE',False,0,0),('IDLE',False,1,2)])
def test_airborne_unknown_or_stale_cannot_edit_ceiling(state,armed,landed,age):
    p=LocalPlatform()
    p.telemetry=dict(flight_state=state,fc_armed=armed,fc_landed=landed)
    p.telemetry_received_s=time.monotonic()-age
    with pytest.raises(ValueError):p.command('set_ceiling',ceiling=3)


def test_ceiling_only_tightens_survey_and_keeps_obstacles():
    m=dict(survey_status='UNVERIFIED',native_body_floor_height_m=1.5,clearance_z_m=.25,
        forbidden=[dict(z_min_m=0,z_max_m=5)],
        **{group:[dict(z_min_m=0,z_max_m=2.5)] for group in ('boundary','altitude_zones','yaw_zones')})
    before=copy.deepcopy(m)
    assert ceiling_map(m,3)==m  # Larger ceiling cannot expand the surveyed map.
    limited=ceiling_map(m,2)
    assert limited['boundary'][0]['z_max_m']==2
    assert limited['survey_status']=='UNVERIFIED' and limited['forbidden']==m['forbidden']
    assert m==before
    with pytest.raises(ValueError,match='clearance'):ceiling_map(m,1.6)
    m['native_body_floor_height_m']=None
    with pytest.raises(ValueError,match='measured'):ceiling_map(m,3)


def test_trial_start_recaptures_px4_body_origin_and_rejects_stale_or_unaligned():
    p=LocalPlatform()
    p.telemetry=dict(flight_state='IDLE',fc_armed=False,fc_landed=1,px4_map_xy_m=[2.1,1.8],x=5.,y=4.,
        preflight={'checks':[dict(code=k,passed=True) for k in ('layout_confirmed','alignment_confirmed','transform','pose','estimator')]})
    p.telemetry_received_s=time.monotonic()
    assert p.automatic_start()==(2.1,1.8)  # Never antenna XY.
    p.telemetry['px4_map_xy_m']=[2.2,1.9]
    request=p.command('start',trial_case='x')
    assert request['planned_launch_xy_m']==[2.2,1.9]
    assert request['route_tasks'][2]['x']==3.2 and request['route_tasks'][2]['y']==1.9
    p.telemetry_received_s=time.monotonic()-1
    with pytest.raises(ValueError):p.command('start',trial_case='hover')
    p.telemetry_received_s=time.monotonic()
    p.telemetry['preflight']['checks'][1]['passed']=False
    with pytest.raises(ValueError):p.command('start',trial_case='hover')
