"""Deterministic MAVLink wire peers; these tests do not simulate flight physics."""
import io
import json
from dataclasses import replace
from pathlib import Path
import struct
from types import SimpleNamespace

import pytest

from drone_demo.mission import MissionConfig
from drone_demo.sitl_navigation import NavigationPlan, SITLNavigationSession
from drone_uwb.integration.sitl.sitl_odometry_contract import SITLOdometrySettings
from drone_uwb.integration.sitl.sitl_target_contract import SITLTargetSettings


d = pytest.importorskip('pymavlink.dialects.v20.common')
CONFIG = MissionConfig(**json.loads((Path(__file__).parents[1]/'config/mission.json')
                                    .read_text(encoding='utf-8')))


def wire(method, *args, system=1, component=1):
    stream = io.BytesIO()
    peer = d.MAVLink(stream, srcSystem=system, srcComponent=component)
    getattr(peer, method)(*args)
    return d.MAVLink(None).parse_buffer(stream.getvalue())[0]


class Peer:
    def __init__(self, *, mode='send', target_settings=None):
        self.host_ns = 1_000_000_000
        self.stream = io.BytesIO()
        self.read_offset = 0
        self.outgoing, self.events = [], []
        self.params = dict(NAV_DLL_ACT=3, COM_DL_LOSS_T=5, COM_DLL_EXCEPT=0,
                           COM_FAIL_ACT_T=0., COM_POS_FS_ACT=0)
        observer = SimpleNamespace(mode='send', dialect=d, fault=None, link_drained=True,
            settings=SITLOdometrySettings(alignment_confirmed=True, px4_reference_confirmed=True),
            connection=SimpleNamespace(mav=d.MAVLink(self.stream, srcSystem=245, srcComponent=191)),
            clock=SimpleNamespace(now_ns=lambda: self.sim_us*1000))
        settings = target_settings or SITLTargetSettings(enabled=True, sitl_session_confirmed=True,
                                                         observation_fusion_confirmed=True)
        self.s = SITLNavigationSession(observer, settings, CONFIG,
            NavigationPlan([[2.59, 1.68]]), mode=mode, write_event=self.events.append,
            host_now_ns=lambda: self.host_ns)

    @property
    def px4_us(self):
        return 10_000_000+(self.host_ns-1_000_000_000)//1000

    @property
    def sim_us(self):
        return self.px4_us+90_000_000

    def drain(self):
        raw = self.stream.getvalue()[self.read_offset:]
        self.read_offset = len(self.stream.getvalue())
        messages = d.MAVLink(None).parse_buffer(raw) or []
        self.outgoing.extend(messages)
        for msg in messages:
            if msg.get_type() == 'TIMESYNC' and msg.tc1 == 0:
                self.s.on_message(wire('timesync_send', self.px4_us*1000, msg.ts1))
        return messages

    def step(self, xy=(2.09, 1.68), *, speed=0., uwb=True, reset=0, flags=None,
             hold=True, armed=True, in_air=True, ack=True, echo=True, dt_ns=50_000_000):
        self.host_ns += dt_ns
        s = self.s
        s.on_message(wire('heartbeat_send', d.MAV_TYPE_QUADROTOR, d.MAV_AUTOPILOT_PX4,
            d.MAV_MODE_FLAG_SAFETY_ARMED if armed else 0,
            (4 << 16) | ((3 if hold else 6) << 24), d.MAV_STATE_ACTIVE, 3))
        s.on_message(wire('extended_sys_state_send', 0,
            d.MAV_LANDED_STATE_IN_AIR if in_air else d.MAV_LANDED_STATE_ON_GROUND))
        s.on_message(wire('gps_global_origin_send', 473979860, 85461920, 200, 3_224_000))
        for name, value in self.params.items():
            is_int = type(value) is int
            raw = struct.unpack('<f', struct.pack('<i', value))[0] if is_int else value
            s.on_message(wire('param_value_send', name.encode(), raw,
                d.MAV_PARAM_TYPE_INT32 if is_int else d.MAV_PARAM_TYPE_REAL32, 5, 0))
        s.clock.poll()
        self.drain()
        s.on_message(wire('odometry_send', self.px4_us, d.MAV_FRAME_LOCAL_NED,
            d.MAV_FRAME_LOCAL_NED, xy[1], xy[0], -.7, [1., 0., 0., 0.],
            0., speed, 0., 0., 0., 0., [.001]*21, [.001]*21, reset,
            d.MAV_ESTIMATOR_TYPE_AUTOPILOT, 0))
        if flags is None:
            flags = (d.ESTIMATOR_ATTITUDE | d.ESTIMATOR_VELOCITY_HORIZ
                     | d.ESTIMATOR_POS_HORIZ_REL | d.ESTIMATOR_POS_HORIZ_ABS)
        s.on_message(wire('estimator_status_send', self.px4_us, flags,
            0., 0., 0., 0., 0., 0., .02, .02))
        if uwb:
            s.on_observation(dict(transmitted=True, time_us=self.sim_us,
                                  sample_age_sim_s=0., wall_residence_s=0.))
        if s.progress is not None:
            packet = s.progress.fields
            if ack:
                s.on_message(wire('command_ack_send', d.MAV_CMD_DO_REPOSITION,
                    d.MAV_RESULT_ACCEPTED, 0, 0, 245, 191))
            if echo:
                s.on_message(wire('position_target_global_int_send', self.px4_us//1000,
                    d.MAV_FRAME_GLOBAL_INT, 0, packet['x'], packet['y'], .7,
                    0., 0., 0., 0., 0., 0., 0., 0.))
        s.tick()
        self.drain()

    def start(self):
        for _ in range(14):
            self.step()
        assert self.s.phase == 'MOVING', self.s.summary()
        assert self.s.sent_count == 1


def commands(peer, number):
    return [msg for msg in peer.outgoing if msg.get_type() in ('COMMAND_INT', 'COMMAND_LONG')
            and msg.command == number]


def test_full_route_and_return_require_dwell_then_observed_landing():
    p = Peer()
    p.start()
    for _ in range(35):
        p.step((2.59, 1.68), speed=.4)
    assert p.s.completed_legs == 0  # Passing the point is not arrival.
    for _ in range(35):
        p.step((2.59, 1.68))
    assert p.s.completed_legs == 1 and p.s.sent_count == 2
    for _ in range(35):
        p.step((2.09, 1.68))
    assert p.s.phase == 'LANDING'
    assert len(commands(p, d.MAV_CMD_NAV_LAND)) == 1
    assert not p.s.summary()['landing_verified']
    p.step(armed=False, in_air=False)
    assert p.s.phase == 'LANDED'
    assert p.s.summary()['route_completed'] and p.s.summary()['landing_verified']
    assert not p.s.summary()['flight_valid'] and not p.s.summary()['fusion_verified']
    assert not commands(p, d.MAV_CMD_COMPONENT_ARM_DISARM)
    assert not commands(p, d.MAV_CMD_NAV_TAKEOFF)
    assessments = [e for e in p.events if e['type'] == 'mission_assessment']
    assert assessments[-1]['gazebo_time_us'] == p.sim_us
    assert assessments[-1]['px4_time_us'] == p.px4_us
    assert assessments[-1]['assessment']['px4_sample_time_us'] == p.px4_us
    assert assessments[-1]['assessment']['position_map_xy_m'] == pytest.approx([2.09, 1.68])


@pytest.mark.parametrize('case', ['monitor', 'disabled', 'wrong_failsafe', 'other_gcs', 'not_hold'])
def test_prerequisites_prevent_navigation_commands(case):
    p = Peer(mode='monitor' if case == 'monitor' else 'send',
             target_settings=SITLTargetSettings() if case == 'disabled' else None)
    if case == 'wrong_failsafe':
        p.params['NAV_DLL_ACT'] = 0
    if case == 'other_gcs':
        p.s.on_message(wire('heartbeat_send', d.MAV_TYPE_GCS, d.MAV_AUTOPILOT_INVALID,
                            0, 0, d.MAV_STATE_ACTIVE, 3, system=250, component=190))
    for _ in range(20):
        p.step(hold=case != 'not_hold')
    assert not commands(p, d.MAV_CMD_DO_REPOSITION)
    assert not commands(p, d.MAV_CMD_NAV_LAND)


def test_uwb_gap_with_valid_px4_requests_braking_once_and_no_auto_resume():
    p = Peer()
    p.start()
    for _ in range(8):
        p.step(uwb=False)
    assert p.s.phase == 'ABORTED'
    pauses = commands(p, d.MAV_CMD_DO_REPOSITION)
    assert len(pauses) == 2 and pauses[-1].x == pauses[-1].y == 2147483647
    assert not commands(p, d.MAV_CMD_NAV_LAND)
    for _ in range(30):
        p.step()
    assert p.s.phase == 'ABORTED' and len(commands(p, d.MAV_CMD_DO_REPOSITION)) == 2


@pytest.mark.parametrize('failure', ['reset', 'invalid_estimator', 'origin', 'observer_fault'])
def test_invalid_px4_state_requests_landing_instead_of_position_hold(failure):
    p = Peer()
    p.start()
    if failure == 'origin':
        p.s.on_message(wire('gps_global_origin_send', 473979861, 85461920, 200, 3_224_001))
    if failure == 'observer_fault':
        p.s.observer.fault = 'gazebo_time_reversed'
    p.step(reset=1 if failure == 'reset' else 0, flags=0 if failure == 'invalid_estimator' else None)
    assert p.s.phase == 'ABORTED'
    assert len(commands(p, d.MAV_CMD_NAV_LAND)) == 1
    assert len(commands(p, d.MAV_CMD_DO_REPOSITION)) == 1


def test_missing_command_evidence_times_out_without_resending_target():
    p = Peer()
    for _ in range(14):
        p.step(ack=False, echo=False)
    for _ in range(45):
        p.step(ack=False, echo=False)
    assert p.s.phase == 'ABORTED' and p.s.reason == 'command_response_timeout'
    assert len(commands(p, d.MAV_CMD_DO_REPOSITION)) == 1


def test_previously_applied_target_stream_gap_aborts_earlier_than_leg_timeout():
    p = Peer()
    p.start()
    for _ in range(12):
        p.step(echo=False)
    assert p.s.phase == 'ABORTED' and p.s.reason == 'px4_target_stream_stale'
    assert len(commands(p, d.MAV_CMD_NAV_LAND)) == 1


def test_newer_telemetry_does_not_starve_buffered_sample_before_clock_reply():
    p = Peer(mode='monitor')
    p.step()
    first = p.s.monitor.last_px4_sample_us
    p.host_ns += 50_000_000
    p.s.clock.poll()
    p.drain()
    for offset in (0, 10000):
        p.s.on_message(wire('odometry_send', p.px4_us+offset, d.MAV_FRAME_LOCAL_NED,
            d.MAV_FRAME_LOCAL_NED, 1.68, 2.09, -.7, [1., 0., 0., 0.],
            0., 0., 0., 0., 0., 0., [.001]*21, [.001]*21, 0, d.MAV_ESTIMATOR_TYPE_AUTOPILOT, 0))
    p.s.update_pose()
    assert p.s.monitor.last_px4_sample_us == first+50000


def test_transport_exception_does_not_retry_ambiguous_target(monkeypatch):
    p = Peer()
    calls = []
    def failed(*args):
        calls.append(args)
        raise OSError('ambiguous transport handoff')
    monkeypatch.setattr(p.s.mav, 'command_int_send', failed)
    for _ in range(14):
        p.step()
    assert len(calls) == 1 and p.s.attempted_count == 1 and p.s.sent_count == 0
    assert p.s.phase == 'ABORTED'
    assert len(commands(p, d.MAV_CMD_NAV_LAND)) == 1


def test_initial_continuity_required_and_gap_restarts_it():
    p = Peer()
    for _ in range(8):
        p.step()
    assert not p.s.sent_count
    for _ in range(6):
        p.step(uwb=False)
    for _ in range(8):
        p.step()
    assert not p.s.sent_count
    for _ in range(5):
        p.step()
    assert p.s.sent_count == 1
