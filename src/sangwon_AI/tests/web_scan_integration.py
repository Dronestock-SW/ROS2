"""Real C++ ledger/runtime boundary with explicit synthetic map and perception."""
import copy
import hashlib
import json
import time
from pathlib import Path
from replay_scan_fixture import bundle
from web_service_integration import Rig
from sangwon_web.ipc import CoreError


def finish(rig, request, timeout=28):
    end=time.monotonic()+timeout
    while time.monotonic()<end:
        rig.core.call('link.update', {'connected': True, 'code': 'OK', 'observed_contract': '1.1-draft.4'})
        result=rig.core.call('command.get', {'control_request_id': request['control_request_id']})
        if result.get('result_revision')==2:
            return result
        time.sleep(.05)
    status=rig.core.call('status')
    raise AssertionError('Synthetic scan did not finish: '+str({k:status['telemetry'][k] for k in ('phase','reason','scan')}))


def execution_modes():
    for mode, attempts, camera, succeeded in [('SCANNER_SUCCESS',1,False,True),('CAMERA_SUCCESS',3,True,True),
            ('QR_UNREADABLE',3,True,False),('MARKER_UNAVAILABLE',0,False,False),('CAMERA_UNAVAILABLE',0,False,False)]:
        raw,config=bundle(outcome=mode)
        with Rig(steps=20,snapshot=raw,overrides=config) as rig:
            rig.connect();ready=rig.prepare()
            assert ready['can_start'] and all(c['applicable'] for c in ready['checks'] if c['check_id'].startswith('QS-'))
            assert ready['validation_scope']=='ALLOWLISTED_SYNTHETIC_MAP_SCAN_ONLY'
            request=rig.command();assert rig.core.call('command',request)['status']=='ACCEPTED'
            response=finish(rig,request)
            assert response['visited']==3 and response['flight_outcome']=='SUCCEEDED',response
            assert response['work_outcome']==('ALL_SUCCEEDED' if succeeded else 'PARTIAL_FAILED'),response
            rows=rig.pending_reports()
            scans=[r for r in rows if r['route']=='scan-task-results']
            assert len(scans)==2,[(r['route'],r['key']) for r in rows]
            first,last=[r['body'] for r in scans]
            assert first['task_status']=='RUNNING' and first['egress_status']=='PENDING'
            assert last['task_status']=='COMPLETED' and last['egress_status']=='COMPLETED'
            assert last['scanner_attempts_started']==attempts and last['camera_qr_used']==camera,last
            assert (last['scan_outcome']=='SUCCEEDED')==succeeded
            if succeeded:
                assert json.loads(last['raw_qr_data'])['code']=='TEST-ITEM-001'
            report=next(r['body'] for r in rows if r['body'].get('type')=='execution_result')
            assert report['scan_summary']=={'total':1,'succeeded':int(succeeded),'failed':int(not succeeded),'not_attempted':0}
            assert report['task_summary']=={'total':3,'succeeded':3-int(not succeeded),'failed':int(not succeeded),'not_attempted':0}
            # Both immutable revisions survive process restart before HTTP receipt.
            before={r['key']:r['sha256'] for r in scans}
            rig.daemon.terminate();rig.daemon.wait(timeout=5)
            rig.daemon=rig.launch('restarted-core',[str(__import__('web_service_integration').DAEMON),'--config',str(rig.path)])
            from web_service_integration import wait_for
            wait_for(lambda:rig.core.call('status')['context']['runtime_session_id']!=ready['context']['runtime_session_id'])
            after={r['key']:r['sha256'] for r in rig.pending_reports() if r['route']=='scan-task-results'}
            assert after==before


def control_after_decode():
    raw,config=bundle()
    with Rig(steps=20,snapshot=raw,overrides=config) as rig:
        rig.connect();rig.prepare();start=rig.command();rig.core.call('command',start)
        until=time.monotonic()+20
        while time.monotonic()<until:
            rig.core.call('link.update',{'connected':True,'code':'OK','observed_contract':'1.1-draft.4'})
            scan=rig.core.call('status')['telemetry'].get('scan')
            if scan and scan['stage']=='RETURN_STAGING':break
            time.sleep(.05)
        else:raise AssertionError('No durable decode before egress')
        pause=rig.command('PAUSE',2);assert rig.core.call('command',pause)['status']=='ACCEPTED'
        paused=finish(rig,pause,timeout=5);assert paused['status']=='COMPLETED'
        status=rig.core.call('status')['telemetry'];assert status['phase']=='PAUSED'
        attempts=status['scan']['scanner_attempts_started'];time.sleep(.1)
        assert rig.core.call('status')['telemetry']['scan']['scanner_attempts_started']==attempts
        resume=rig.command('RESUME',3);assert rig.core.call('command',resume)['status']=='ACCEPTED'
        assert finish(rig,resume,timeout=5)['status']=='COMPLETED'
        cancel=rig.command('CANCEL',4);assert rig.core.call('command',cancel)['status']=='ACCEPTED'
        response=finish(rig,start)
        assert response['flight_outcome']=='CANCELED' and response['work_outcome']=='INCOMPLETE'
        rows=rig.pending_reports();reports=[r['body'] for r in rows if r['route']=='scan-task-results']
        assert len(reports)==2 and reports[-1]['scan_outcome']=='SUCCEEDED'
        assert reports[-1]['task_status']=='FAILED' and reports[-1]['egress_status']=='PREEMPTED'
        summary=next(r['body'] for r in rows if r['body'].get('type')=='execution_result')
        assert summary['scan_summary']=={'total':1,'succeeded':1,'failed':0,'not_attempted':0}
        assert summary['task_summary']=={'total':3,'succeeded':1,'failed':1,'not_attempted':1}


def rejection_cases():
    raw,config=bundle()
    cases=[]
    invalid=copy.deepcopy(config);invalid['replay_profile_artifacts']['scan-demo@r1']='{}'
    cases.append((raw,invalid,'PROFILE_HASH_MISMATCH'))
    invalid=copy.deepcopy(config);invalid.pop('replay_scan_source')
    cases.append((raw,invalid,'SCAN_ADAPTER_NOT_CONFIGURED'))
    for code,change in [('APPROVAL_EXPIRED',lambda s:s['approvals'][0].update(valid_until='2026-01-01T00:00:00Z')),
        ('UNSUPPORTED_LABEL_NORMAL',lambda s:s['labels'][0].update(outward_normal_map=[0,0,1])),
        ('INVALID_MAP_GEOMETRY',lambda s:s['scan_workspaces'][0].update(polygon_xy_m=[[2.5,.5],[4.5,2.5],[4.5,.5],[2.5,2.5]])),
        ('SCAN_WORKSPACE_TOO_SMALL',lambda s:s['labels'][0]['position_m'].update(x=5.5)),
        ('ROUTE_OBSTRUCTED',lambda s:s['map']['obstacles'][0].update(polygon_xy_m=[[1,1],[2,1],[2,2],[1,2]]))]:
        data=json.loads(raw);change(data);altered=json.dumps(data,sort_keys=True,separators=(',',':')).encode()
        cfg=copy.deepcopy(config);cfg['approved_replay_snapshot_sha256']=[hashlib.sha256(altered).hexdigest()]
        cases.append((altered,cfg,code))
    # An arbitrary valid-looking fixture is still not authorized by its hash.
    invalid=copy.deepcopy(config);invalid['approved_replay_snapshot_sha256']=[]
    cases.append((raw,invalid,'UNAPPROVED_REPLAY_FIXTURE'))
    for raw,config,expected in cases:
        with Rig(snapshot=raw,overrides=config) as rig:
            rig.connect()
            try:rig.prepare()
            except CoreError as error:assert expected in str(error),(expected,str(error))
            else:raise AssertionError('Invalid synthetic bundle was accepted: '+expected)
            assert not rig.core.call('status')['readiness']['can_start']


def polygon_compilation():
    raw,config=bundle()
    data=json.loads(raw)
    data['scan_workspaces'][0]['polygon_xy_m'][1]=[4.4,.6]
    data['map']['altitude_zones'][0]['polygon_xy_m']=[[0,0],[5.8,0],[6,.2],[6,5],[0,5]]
    data['map']['yaw_validation_zones'][0]['polygon_xy_m']=[[0,0],[6,0],[6,5],[4,5],[4,4],[3,4],[3,5],[0,5]]
    raw=json.dumps(data,sort_keys=True,separators=(',',':')).encode()
    config['approved_replay_snapshot_sha256']=[hashlib.sha256(raw).hexdigest()]
    with Rig(steps=20,snapshot=raw,overrides=config) as rig:
        rig.connect();assert rig.prepare()['can_start']
        request=rig.command();assert rig.core.call('command',request)['status']=='ACCEPTED'
        response=finish(rig,request)
        assert response['visited']==3 and response['work_outcome']=='ALL_SUCCEEDED',response


if __name__=='__main__':
    execution_modes();control_after_decode();rejection_cases();polygon_compilation()
    print('PASS synthetic scan actions, durable revisions, restart, map/profile/approval rejection')
