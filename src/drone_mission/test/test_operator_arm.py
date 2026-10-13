from dataclasses import replace
from datetime import datetime, timezone
import time

import pytest

from drone_mission.contracts import Settings
from drone_mission.session import Snapshot
from drone_mission.operator_arm import arm_blocker, arm_request
from drone_mission.local_web import LocalPlatform
from drone_mission.bridge_status import normalize_bridge


def ready_sample():
    return Snapshot(connected=True, state_age_s=0, landed=1, landed_age_s=0,
        mode='POSCTL', rc_valid=True, rc_age_s=0, rc_mapping_valid=True,
        battery=.8, battery_age_s=0, command_services_ready=True)


def test_manual_arm_does_not_require_or_claim_autonomous_route_readiness():
    s=Settings(execute=True)
    assert arm_blocker(s,ready_sample(),idle=True,recording_ok=True)==''
    assert not s.alignment_confirmed and not s.fusion_confirmed


@pytest.mark.parametrize('change', [dict(mode='AUTO.TAKEOFF'),dict(armed=True),dict(landed=2),
    dict(state_age_s=2),dict(rc_age_s=2),dict(rc_valid=False),dict(battery=.2),dict(battery=float('nan')),
    dict(command_services_ready=False),dict(rc_mapping_valid=False)])
def test_manual_arm_refuses_unsafe_or_stale_state(change):
    assert arm_blocker(Settings(execute=True),replace(ready_sample(),**change),idle=True,recording_ok=True)


def test_execution_logging_and_idle_are_required():
    assert arm_blocker(Settings(),ready_sample(),idle=True,recording_ok=True)
    assert arm_blocker(Settings(execute=True),ready_sample(),idle=False,recording_ok=True)
    assert arm_blocker(Settings(execute=True),ready_sample(),idle=True,recording_ok=False)


def test_request_is_bound_to_current_executor_device_and_short_expiry():
    now=1_791_417_600.;s=Settings()
    p=dict(ok=True,contract_version='1.0',drone_id=s.drone_id,anchor_layout_id=s.layout_id,
        operator_session_id='current',control_action='arm',control_request_id='one-click',
        control_requested_at=datetime.fromtimestamp(now,timezone.utc).isoformat())
    assert arm_request(p,s,'current',now)=='one-click'
    for changes in (dict(operator_session_id='old'),dict(drone_id='6'),dict(control_action='start')):
        with pytest.raises(ValueError):arm_request(dict(p,**changes),s,'current',now)
    with pytest.raises(ValueError):arm_request(p,s,'current',now+4)
    with pytest.raises(ValueError):arm_request(p,s,'current',now-1)


def test_arm_retry_is_durable_and_does_not_create_or_replay_a_mission(tmp_path):
    path=tmp_path/'mission.sqlite3';s=Settings(full_mission=True)
    p=LocalPlatform(settings=s,mission_file=path)
    p.telemetry=dict(fc_connected=True,fc_armed=False,fc_landed=1,flight_state='IDLE',
        arm_request_available=True,operator_session_id='current')
    p.telemetry_received_s=time.monotonic()
    assert p.can_arm() and not p.can_start()
    body=dict(client_request_id='one-click',operator_session_id='current')
    first=p.arm(body)
    assert not p.can_arm() and not p.can_start()
    assert p.arm(body)==first and not p.store.active()
    p.store.close()
    reopened=LocalPlatform(settings=s,mission_file=path)
    assert reopened.arm(body)==first
    assert reopened.assignment['control_action'] is None
    reopened.store.close()


def test_local_odometry_adapter_preserves_unverified_flight_flags():
    s=Settings(bridge_topic='/uwb/local_odometry_status',bridge_host_boot_id='boot')
    v=dict(settings=dict(host_boot_id='boot'),counts=dict(published=10),last_sample_stamp_ns=100,
        last_send_age_s=.01,timesync_ready=True,gate='ready',dynamic_alignment_verified=False,
        flight_verified=False)
    n=normalize_bridge(v,s)
    assert n['published']==10 and n['last_observation_stamp_ns']==100
    assert n['flight_verified'] is False and n['dynamic_alignment_verified'] is False
    assert normalize_bridge(v,replace(s,bridge_host_boot_id='old'))['gate']!='ready'
    assert normalize_bridge(dict(v,last_send_age_s=2.),s)['gate']!='ready'


@pytest.mark.parametrize('action',['takeoff','takeoff_land'])
def test_separate_takeoff_uses_mission_ttl_without_restamping(action):
    s=Settings();now=1791417600.
    p=dict(ok=True,contract_version='1.0',drone_id=s.drone_id,anchor_layout_id=s.layout_id,
        operator_session_id='current',control_action=action,control_request_id='one-click',
        control_requested_at=datetime.fromtimestamp(now,timezone.utc).isoformat())
    assert arm_request(p,s,'current',now+4,action=action)=='one-click'
    with pytest.raises(ValueError):arm_request(p,s,'current',now+30.01,action=action)
    with pytest.raises(ValueError):arm_request(p,s,'old',now+4,action=action)
    with pytest.raises(ValueError):arm_request(p,s,'current',now-1,action=action)
