"""Real C++ BT + HTTP START + sole Python executor. The FC is synthetic."""
from dataclasses import replace
from contextlib import contextmanager, ExitStack
import asyncio
from http.server import ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time
from urllib.request import Request, urlopen

import pytest
from drone_mission.cpp_mission_ai import CppMissionAI
from drone_mission.local_web import LocalPlatform
from drone_mission.local_xy import LocalXYMission
from test_local_xy import sample
from test_anchor_trial import config


@contextmanager
def websocket_peer(platform):
    from websockets.asyncio.server import serve
    from websockets.sync.client import connect
    ready=threading.Event();context={}
    async def run():
        context['loop']=asyncio.get_running_loop();context['stop']=asyncio.Event()
        async with serve(platform.websocket,'127.0.0.1',0) as server:
            context['port']=server.sockets[0].getsockname()[1];ready.set()
            await context['stop'].wait()
    thread=threading.Thread(target=lambda:asyncio.run(run()),daemon=True);thread.start()
    assert ready.wait(3),'synthetic websocket server startup failed'
    try:
        with connect(f"ws://127.0.0.1:{context['port']}/ws/drones/{platform.drone_id}/",open_timeout=2) as peer:
            yield peer
    finally:
        context['loop'].call_soon_threadsafe(context['stop'].set);thread.join(3)


class TelemetrySender:
    """Production uses a separate platform process; never block the FC loop on WS."""
    def __init__(self,peer):
        self.peer=peer;self.pending=None;self.stop=False;self.error=None;self.condition=threading.Condition()
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def submit(self,value):
        with self.condition:self.pending=value;self.condition.notify()
    def run(self):
        try:
            while True:
                with self.condition:
                    self.condition.wait_for(lambda:self.stop or self.pending is not None)
                    if self.pending is None and self.stop:return
                    value=self.pending;self.pending=None
                self.peer.send(json.dumps(value))
        except Exception as exc:self.error=exc
    def __enter__(self):return self
    def __exit__(self,*args):
        with self.condition:self.stop=True;self.condition.notify()
        self.thread.join(2)
        assert not self.thread.is_alive(),'telemetry sender stalled'
        assert self.error is None,self.error


def binary():
    value = os.environ.get('SANGWON_CPP_AI_BINARY')
    if sys.platform != 'linux' or not value:
        pytest.skip('actual Linux C++ binary required; run field_mission_ai_contracts and this file on Jetson')
    return value


@pytest.mark.parametrize('status,allowed', [
    ({},False),({'backend':'python_test','connected':True},False),
    ({'backend':'sangwon_cpp_bt','connected':False},False),
    ({'backend':'sangwon_cpp_bt','connected':True},True)])
def test_cpp_field_web_requires_live_backend_for_START_and_ARM(tmp_path,status,allowed):
    cfg=replace(config(command_frame='local_xy',mission_ai='cpp_bt'),expected_mis_takeoff_alt_m=1.7)
    p=LocalPlatform(cfg.drone_id,cfg.layout_id,settings=cfg,mission_file=tmp_path/'missions.sqlite3')
    try:
        p.telemetry=dict(fc_connected=True,fc_armed=False,fc_landed=1,flight_state='IDLE',
            preflight={'checked_inputs_passed':True},arm_request_available=True,mission_ai=status)
        p.telemetry_received_s=time.monotonic();p.site['ceiling_height_m']=3.
        assert p.can_start() is allowed and p.can_arm() is allowed
    finally:p.close()


@pytest.mark.parametrize('scenario',['complete','native_takeoff','cpp_loss','rc_takeover','lidar_gap','lidar_loss','lidar_sdk_gap'])
def test_http_hover_uses_cpp_bt_from_start_through_confirmed_END(tmp_path,scenario):
    cfg=replace(config(command_frame='local_xy', mission_ai='cpp_bt'),expected_mis_takeoff_alt_m=1.7)
    native=scenario=='native_takeoff'
    if native:cfg=replace(cfg,start_mode='takeoff_hold')
    use_lidar=scenario.startswith('lidar_')
    if use_lidar:
        sys.path.insert(0,str(Path(__file__).resolve().parents[2]/'sangwon_AI/tests'))
        from lidar_relative_contracts import CONFIG,scan,sdk_state
        lidar_config=dict(CONFIG)
        if scenario=='lidar_sdk_gap':lidar_config['input_source']='sdk_pointcloud'
        cfg=replace(cfg,lidar_gap=lidar_config)
    ai=CppMissionAI(binary())
    mission=LocalXYMission(cfg);mission.mission_ai=ai
    p=LocalPlatform(cfg.drone_id,cfg.layout_id,settings=cfg,mission_file=tmp_path/'missions.sqlite3')
    now=time.monotonic()
    for t in (now-3.1,now-2.,now-.1):
        mission.tick(sample(t,takeoff_alt_m=1.7),t,time.time(),time.time_ns())
    if native:
        now=time.monotonic()
        mission.begin_native(dict(action='takeoff',request_id='synthetic-native',requested_at='synthetic'),sample(now,takeoff_alt_m=1.7),now)
        mode='POSCTL';armed=False;landed=1;alt=.1;native_commands=[]
        deadline=now+8
        while time.monotonic()<deadline:
            now=time.monotonic();mission.last_web_s=now
            actions=mission.tick(sample(now,takeoff_alt_m=1.7,mode=mode,armed=armed,landed=landed,altitude_enu_m=alt),now,time.time(),time.time_ns())
            assert mission.local_setpoint() is None
            for a in actions:
                native_commands.append(a['kind']);mission.command_result(a['token'],True,now)
                if a['kind']=='takeoff_mode':mode='AUTO.TAKEOFF'
                elif a['kind']=='arm':armed=True
            if mission.phase in ('NATIVE_WAIT_ASCENT','NATIVE_CLIMB'):mode='AUTO.LOITER';landed=2;alt=1.8
            if mission.phase=='HOLD_READY':break
            time.sleep(.05)
        assert mission.phase=='HOLD_READY' and native_commands==['takeoff_mode','arm']
        assert mission.assignment is None and ai.plan_ref is None
    checks=dict(checked_inputs_passed=True,checks=[dict(code=k,passed=True) for k in ('layout_confirmed','transform','pose','estimator')])
    p.telemetry=dict(fc_connected=True,fc_armed=False,fc_landed=1,flight_state='IDLE',
        px4_map_xy_m=[2.,2.],preflight=checks,mission_ai=ai.status())
    if native:p.telemetry.update(fc_armed=True,fc_landed=2,fc_mode='AUTO.LOITER',flight_state='HOLD_READY',native_takeoff=dict(can_start=True),home_xy_m=mission.status()['home_xy_m'])
    p.telemetry_received_s=time.monotonic();p.site['ceiling_height_m']=3.
    server=ThreadingHTTPServer(('127.0.0.1',0),p.http_handler())
    web=threading.Thread(target=server.serve_forever,daemon=True);web.start()
    stack=ExitStack()
    try:
        data=json.dumps(dict(action='start',trial_case='hover',client_request_id='cpp-http-hover')).encode()
        request=Request('http://127.0.0.1:'+str(server.server_port)+'/local/command',data=data,
            headers={'Content-Type':'application/json'},method='POST')
        with urlopen(request,timeout=2) as response:assignment=json.load(response)
        with urlopen(request,timeout=2) as response:assert json.load(response)==assignment
        assert mission.submit(assignment,time.monotonic(),time.time())
        armed=native;mode='AUTO.LOITER' if native else 'POSCTL';landed=2 if native else 1;alt=1.8 if native else .1;xy=(2.,2.)
        commands=[];phases=[];trace=[];prior=time.monotonic();start=prior;injected=False;lost_at=None;cancelled=False
        last_uwb=prior;gap_start=None;gap_seen=False;uwb_history=[]
        peer=stack.enter_context(websocket_peer(p));sender=stack.enter_context(TelemetrySender(peer))
        # Includes the real 10s dwell, ascent/landing, and asynchronous C++/WS
        # handoffs. Keep a bound without truncating the full loaded-Jetson run.
        while time.monotonic()-start<45:
            now=time.monotonic();dt=now-prior;prior=now;ns=time.time_ns();previous_alt=alt
            goal=mission.local_setpoint()
            if goal:
                xy=tuple(goal['xy'])
                alt+=max(-.5*dt,min(.5*dt,goal['altitude_enu_m']-alt))
                mission.applied_local_target(xy,goal['altitude_enu_m'],ns)
            if armed and alt>.15:landed=2
            if mode=='AUTO.LAND':
                alt=max(.1,alt-.5*dt)
                if alt<=.100001:armed=False;landed=1
            s=sample(now,takeoff_alt_m=1.7,armed=armed,mode=mode,landed=landed,xy=xy,uwb_xy=xy,
                altitude_enu_m=alt,vertical_speed_m_s=(alt-previous_alt)/dt)
            if use_lidar:
                stamp=math.floor(now*10)/10-.005
                s.lidar_scan=(sdk_state(stamp-.0719)['lidar_scan'] if scenario=='lidar_sdk_gap' else scan(stamp))
                s.lidar_scan['age_s']=now-stamp
                s.lidar_scan['first_beam_age_s']=now-s.lidar_scan['first_beam_stamp_ns']/1e9
                if gap_start is None or now-gap_start>.6:last_uwb=now
                s.uwb_stamp_ns=round(last_uwb*1e9);s.uwb_age_s=s.bridge_observation_age_s=now-last_uwb
                if last_uwb==now:uwb_history.append((s.uwb_stamp_ns,xy));uwb_history=uwb_history[-40:]
                from drone_mission.lidar_input import nearest_uwb
                try:
                    anchor,anchor_xy=nearest_uwb(uwb_history,s.lidar_scan['stamp_ns'])
                    s.lidar_scan.update(anchor_uwb_stamp_ns=anchor,anchor_uwb_xy=anchor_xy,anchor_uwb_age_s=now-anchor/1e9)
                except ValueError:pass
                if scenario=='lidar_loss' and gap_start is not None:s.lidar_scan['ranges']=[None]*720
            actions=mission.tick(s,now,time.time(),ns)
            mission.submit(assignment,now,time.time())
            if not phases or phases[-1]!=mission.phase:phases.append(mission.phase)
            for a in actions:
                commands.append(a['kind']);mission.command_result(a['token'],True,now)
                if a['kind']=='offboard_mode':mode='OFFBOARD'
                elif a['kind']=='arm':armed=True
                elif a['kind']=='land':mode='AUTO.LAND'
            status=mission.status();trace.append(dict(elapsed_s=now-start,state=mission.phase,ai=status['mission_ai'],xy=list(xy),height_m=alt,
                uwb_age_s=s.uwb_age_s,horizontal_observation=status.get('horizontal_observation')))
            gap_seen=gap_seen or status.get('horizontal_observation',{}).get('gap_supported',False)
            sender.submit(dict(status,type='telemetry',drone_id=cfg.drone_id,
                flight_state=mission.phase,fc_connected=True,fc_armed=armed,fc_landed=landed))
            if mission.phase=='DWELL' and not injected:
                if scenario=='cpp_loss':ai.process.kill();lost_at=now
                elif scenario=='rc_takeover':mode='ALTCTL'
                elif use_lidar:gap_start=now
                injected=True
            if lost_at is not None and now-lost_at>.5 and not cancelled:
                assert mission.local_setpoint() is not None and 'land' not in commands
                assert mission.completed_waypoints==0
                request=Request('http://127.0.0.1:'+str(server.server_port)+'/local/command',
                    data=json.dumps(dict(action='land',client_request_id='explicit-cancel')).encode(),
                    headers={'Content-Type':'application/json'},method='POST')
                with urlopen(request,timeout=2) as response:assignment=json.load(response)
                cancelled=True
            if mission.phase in mission.TERMINAL:break
            time.sleep(.05)
        if scenario=='lidar_loss':
            # This legacy mission diagnostic is not the new required PX4 LiDAR
            # takeoff source. A 0.6s UWB gap recovers inside the FC-owned grace.
            assert not gap_seen
        if scenario not in ('complete','native_takeoff','lidar_gap','lidar_sdk_gap','lidar_loss'):
            assert mission.phase==('FAILED' if scenario=='cpp_loss' else 'PILOT_OVERRIDE'),(mission.phase,mission.reason)
            assert commands==(['offboard_mode','arm','land'] if scenario=='cpp_loss' else ['offboard_mode','arm'])
            assert mission.local_setpoint() is None and not mission.status()['mission_complete']
            assert not mission.submit({**assignment,'control_action':'start','control_request_id':'new-start'},time.monotonic(),time.time())
            return
        assert mission.phase=='END',(mission.reason,ai.status(),phases)
        if use_lidar and scenario!='lidar_loss':assert gap_seen,'a fresh independent LiDAR must support the injected 0.6s gap'
        assert commands==(['offboard_mode','land'] if native else ['offboard_mode','arm','land'])
        if native:assert all(row['height_m']<=1.800001 for row in trace)
        assert mission.completed_waypoints==1 and mission.home==(2.,2.)
        assert mission.home_verified and mission.landing_verified and mission.status()['mission_complete']
        assert ai.status()['phase']=='END' and ai.status()['plan_ref'].startswith('cpp-bt-local-xy:')
        assert set(phases)>=({'PREPARING','HANDOFF','MOVING','DWELL','RETURNING','LANDING','END'} if native else {'PREPARING','ARMING','TAKING_OFF','MOVING','DWELL','RETURNING','LANDING','END'})
        assert all(math.dist(row['xy'],(2.,2.))==0 for row in trace)
        end=time.monotonic()+2
        while True:
            with urlopen('http://127.0.0.1:'+str(server.server_port)+'/local/status',timeout=2) as response:
                returned=json.load(response)['telemetry']
            if returned.get('flight_state')=='END':break
            assert time.monotonic()<end;time.sleep(.02)
        assert returned['mission_ai']['phase']=='END' and returned['mission_complete'] is True
        output=os.environ.get('SANGWON_CPP_AI_EVIDENCE')
        if output:
            if use_lidar:output=str(Path(output).with_name('http-cpp-lidar-gap.json'))
            Path(output).write_text(json.dumps(dict(scope='SYNTHETIC_FC_HTTP_CPP_BT_EXECUTOR',
                physical_flight=False,commands=commands,phases=phases,trace=trace,
                mission_complete=True,binary_sha256=ai.binary_sha256),indent=2)+'\n',encoding='utf-8')
    finally:
        output=os.environ.get('SANGWON_CPP_AI_EVIDENCE')
        if output and 'trace' in locals():
            evidence=json.dumps(dict(
                scope='SYNTHETIC_FC_HTTP_CPP_BT_EXECUTOR',scenario=scenario,physical_flight=False,
                state=mission.phase,reason=mission.reason,commands=commands,trace=trace,ai=ai.status()),indent=2)+'\n'
            archive=Path(output).parent/('execution-'+ai.session_id)
            archive.mkdir(exist_ok=False)
            (archive/('executor-'+scenario+'.json')).write_text(evidence,encoding='utf-8')
            Path(output).with_name('executor-'+scenario+'.json').write_text(evidence,encoding='utf-8')
        stack.close()
        server.shutdown();server.server_close();web.join(2);ai.close();p.close()


def test_actual_cpp_stall_stops_decisions_without_python_fallback():
    ai=CppMissionAI(binary())
    try:
        cfg=replace(config(command_frame='local_xy',mission_ai='cpp_bt'),expected_mis_takeoff_alt_m=1.7)
        request=dict(request_id='stall',waypoints=[dict(id='P1',type='hover',xy=(2.,2.),dwell_s=10.)])
        s=sample(time.monotonic(),takeoff_alt_m=1.7)
        end=time.monotonic()+2
        while not ai.prepare(request,cfg,s,3.):
            assert time.monotonic()<end;time.sleep(.01)
        os.kill(ai.process.pid,signal.SIGSTOP)
        ai.step(s,time.monotonic(),executor_phase='PREPARING',feedback_age_s=0.)
        end=time.monotonic()+2
        while ai.status()['error'] is None:
            assert time.monotonic()<end;time.sleep(.01)
        assert 'cpp_ai_reply_timeout' in ai.status()['error']
        with pytest.raises(ValueError,match='cpp_ai_reply_timeout'):
            ai.step(s,time.monotonic(),executor_phase='PREPARING',feedback_age_s=0.)
    finally:ai.close()
