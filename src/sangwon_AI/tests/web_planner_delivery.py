"""A normal saved planner document is received by C++; it never grants START."""
import copy
import hashlib
from web_service_integration import Rig, wait_for, encode, decode, stop
from sangwon_web.common import utc_now
from sangwon_web.ipc import CoreError


def payload(drone):
    plan={'contract_version':'1.1-draft.4','type':'mission_plan','scope':'PLANNING_ONLY',
        'plan_id':'plan-fixture-001','drone_id':drone,'mission_db_id':1,'mission_code':'fixture',
        'coordinate_frame':{'id':'WAREHOUSE_MAP','length_unit':'m'},'start_mode':'AUTO_TAKEOFF','takeoff_z_m':1.1,
        'route_tasks':[{'type':'waypoint','x':1,'y':2,'z':1.2,'yaw_deg':10},{'type':'hover','x':2,'y':3,'z':1.8,'hold_s':2}],
        'labels':[],'map':{'width_m':10,'height_m':10},'execution_eligible':False,'physical_flight_approval':False,
        'blocking_reasons':['EXECUTION_SNAPSHOT_REQUIRED','PX4_UWB_VALIDATION_REQUIRED']}
    return plan


def session(rig):
    state=rig.core.call('status');ctx=state['context']
    rig.core.call('session.set',{'contract_version':'1.1-draft.4','profile':state['profile'],'drone_id':state['drone_id'],
        'flight_authority':False,'allowed_execution':'NONE','boot_id':ctx['boot_id'],'runtime_session_id':ctx['runtime_session_id'],
        'server_time':utc_now(),'control_session_id':'planner-fixture-session'})
    return {**{k:ctx[k] for k in ('boot_id','runtime_session_id')},'control_session_id':'planner-fixture-session'}


def request(plan,context):
    text=encode(plan).decode();raw=text.encode()
    return {'context':context,'plan_text':text,'plan_ref':{'plan_id':plan['plan_id'],'sha256':hashlib.sha256(raw).hexdigest(),
        'byte_length':len(raw),'content_url':f"/api/drones/{plan['drone_id']}/mission-plan/"}}


def main():
    with Rig(profile='HOST_OBSERVE') as rig:
        context=session(rig);plan=payload(rig.config['drone_id']);value=request(plan,context)
        result=rig.core.call('plan.receive',value)
        assert result['state']=='RECEIVED' and result['task_count']==2
        state=rig.core.call('status')
        assert not state['readiness']['can_start'] and not state['physical_output_enabled']
        assert state['context']['preparation_id'] is None and state['telemetry']['px4']['connected'] is False
        rows=rig.core.call('outbox.list');assert len(rows)==1 and rows[0]['route']=='mission-plan-receipts'
        for _ in range(2):assert rig.core.call('plan.receive',value)==result
        assert rig.core.call('outbox.list')==rows
        bad=[]
        for field,other in [('drone_id','OTHER'),('scope','EXECUTION'),('execution_eligible',True),('physical_flight_approval',True)]:
            candidate=copy.deepcopy(plan);candidate[field]=other;bad.append(request(candidate,context))
        candidate=copy.deepcopy(plan);candidate['route_tasks'][0]['z']=31;bad.append(request(candidate,context))
        candidate=copy.deepcopy(plan);candidate['route_tasks'][0]['yaw_deg']=180;bad.append(request(candidate,context))
        candidate=copy.deepcopy(plan);candidate['route_tasks'][0]['type']='scan';candidate['route_tasks'][0]['label_point_id']=999;bad.append(request(candidate,context))
        candidate=copy.deepcopy(plan);candidate['route_tasks'][0]['x']=2;bad.append(request(candidate,context))
        candidate=copy.deepcopy(value);candidate['plan_ref']['sha256']='0'*64;bad.append(candidate)
        candidate=copy.deepcopy(value);candidate['context']['boot_id']='old';bad.append(candidate)
        for candidate in bad:
            try:rig.core.call('plan.receive',candidate)
            except CoreError:pass
            else:raise AssertionError('Invalid planner document accepted')
        assert rig.core.call('outbox.list')==rows
        stop(rig.daemon)
        rig.daemon=rig.launch('core-restarted',[str(rig.daemon.args[0]),'--config',str(rig.path)])
        wait_for(lambda:rig.core.call('status'))
        assert rig.core.call('status')['mission_plan'] is None
        assert rig.core.call('outbox.list')==rows, 'Unacknowledged receipt must survive restart'
        context=session(rig);new=rig.core.call('plan.receive',request(plan,context))
        assert new['context']['runtime_session_id']!=result['context']['runtime_session_id']
        assert len(rig.core.call('outbox.list'))==2, 'Restart receipt must carry the new runtime context'
        assert not rig.core.call('status')['readiness']['can_start']
    print('PASS planner receive/persistence/deduplication/10 invalid cases/restart; no preparation or physical output')


if __name__=='__main__':main()
