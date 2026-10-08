"""Exercise the real C++ field-map compiler, including collision rejection."""
import copy
import json
import subprocess
import sys
v=dict(polygon_xy_m=[[0,0],[6.3,0],[6.3,4.6],[0,4.6]],z_min_m=0,z_max_m=3)
source=dict(schema='sangwon-native-plan/1',native_height_m=1.3,mag_type=6,max_leg_m=1.,launch_xy_m=[2,2],
    map=dict(frame='UWB_ANCHOR_LOCAL',unit='meter',layout_id='layout',
        survey_status='VIRTUAL',launch_body_floor_height_m=.3,native_body_floor_height_m=1.3,
        altitude_policy='px4_MIS_TAKEOFF_ALT',native_height_m=1.3,clearance_xy_m=.5,clearance_z_m=.15,
        boundary=[v],altitude_zones=[v],yaw_zones=[v],forbidden=[]),
    assignment=dict(anchor_layout_id='layout',route_tasks=[dict(id='P1',x=2.4,y=2.),
        dict(id='S1',type='scan',x=3.2,y=2.2,staging_xy_m=[2.7,2.2],yaw_deg=15,
             marker_id=7,label_id='LABEL7',calibration_ref='camera',mounting_ref='mount')]))
def run(data):
    return subprocess.run([sys.argv[1]],input=json.dumps(data),text=True,capture_output=True,timeout=2)
r=run(source);assert r.returncode==0,r.stderr
plan=json.loads(r.stdout)
assert plan['validated'] and plan['flight_authority'] is False
assert plan['route_tasks'][1]['x']==3.2 and plan['route_tasks'][1]['staging_xy_m']==[2.7,2.2]
assert plan['route_tasks'][1]['path_validation_ref'].startswith('native-map-sha256:')
low=copy.deepcopy(source)
for group in ('boundary','altitude_zones','yaw_zones'):low['map'][group][0]['z_max_m']=1.55
assert run(low).returncode==0  # Floor reference is measured; no launch+climb inference.
cases=[]
def bad(change,code):
    x=copy.deepcopy(source);change(x);cases.append((x,code))
bad(lambda x:x.update(native_height_m=1.7),'NATIVE_HEIGHT_MISMATCH')
bad(lambda x:x['map'].update(forbidden=[dict(polygon_xy_m=[[2.45,1.8],[2.55,1.8],[2.55,2.4],[2.45,2.4]],z_min_m=0,z_max_m=3)]),'ROUTE_OBSTRUCTED')
bad(lambda x:x['map'].update(boundary=[]),'COMPLETE_MAP_REQUIRED')
bad(lambda x:x['assignment']['route_tasks'][0].update(x=4),'NATIVE_LEG_TOO_LONG')
bad(lambda x:x['assignment']['route_tasks'][0].update(z=1.3),'COMPANION_Z_FORBIDDEN')
bad(lambda x:x['assignment']['route_tasks'][1].update(id='P1'),'TASK_ID_REQUIRED')
bad(lambda x:x['assignment'].update(anchor_layout_id='other'),'LAYOUT_MISMATCH')
bad(lambda x:x['assignment']['route_tasks'][1].update(staging_xy_m=[.55,2]),'ALTITUDE_ZONE_UNCOVERED')
bad(lambda x:x['map'].update(frame='PX4_LOCAL_ENU'),'FRAME_MISMATCH')
bad(lambda x:x['map'].update(native_body_floor_height_m=None),'INVALID_NUMBER')
bad(lambda x:x.update(mag_type=0),'HEADING_HEIGHT_POLICY_INCOMPATIBLE')
for data,code in cases:
    result=run(data);assert result.returncode==2 and code in result.stderr,(code,result.stderr)
print('Native AI: 2 accepted routes and 11 rejected unsafe contracts')
