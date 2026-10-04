"""Exercise real UWB solvers through the runtime handoff and failure gates."""
from collections import deque
from dataclasses import replace
import json
import math
import struct
from types import SimpleNamespace

import numpy as np
import pytest

from drone_uwb.integration.gazebo_clock import ClockUnavailable, GazeboSimulationClock
from drone_uwb.integration.sitl_link_probe import PARAMETERS
from drone_uwb.integration.sitl_observer import SITLObserver, SITLObserverSettings, log_value
from drone_uwb.integration.sitl_odometry_contract import SITLOdometrySettings
from drone_uwb.processing.experiments.gazebo_live_shadow import GazeboLiveShadow
from test_gazebo_live_shadow import configuration, cycle, sensor_rows


DIALECT = SimpleNamespace(MAV_AUTOPILOT_PX4=12, MAV_PARAM_TYPE_INT32=6,
    MAV_PARAM_TYPE_REAL32=9, MAV_ESTIMATOR_TYPE_AUTOPILOT=8,
    MAV_FRAME_LOCAL_NED=1, MAV_FRAME_BODY_FRD=12, MAV_ESTIMATOR_TYPE_VISION=2,
    MAVLink_timesync_message=SimpleNamespace(fieldnames=['tc1', 'ts1']))
PARAMS = dict(EKF2_EV_CTRL=1, EKF2_EV_NOISE_MD=0, EKF2_EV_DELAY=0.,
    EKF2_EV_POS_X=0., EKF2_EV_POS_Y=0., EKF2_EV_POS_Z=0., EKF2_EVP_NOISE=.1,
    EKF2_GPS_CTRL=7, EKF2_OF_CTRL=1, EKF2_RNG_CTRL=2, EKF2_HGT_REF=2)


class Message(SimpleNamespace):
    def get_type(self):
        return self.kind

    def get_srcSystem(self):
        return getattr(self, 'source_system', 1)

    def get_srcComponent(self):
        return 1

    def to_dict(self):
        return dict(mavpackettype=self.kind, **{key: value for key, value in vars(self).items()
                                              if key not in ('kind', 'source_system')})


class Sender:
    def __init__(self):
        self.calls = []

    def param_request_read_send(self, *args):
        self.calls.append(('parameter_read', args))

    def timesync_send(self, *args):
        self.calls.append(('timesync', args))

    def odometry_send(self, *args):
        self.calls.append(('odometry', args))


class Connection:
    def __init__(self):
        self.mav, self.messages = Sender(), deque()

    def recv_match(self, *, blocking):
        assert not blocking
        return self.messages.popleft() if self.messages else None


def parameter(name, value):
    is_int = type(value) is int
    raw = struct.unpack('<f', struct.pack('<i', value))[0] if is_int else value
    return Message(kind='PARAM_VALUE', param_id=name, param_value=raw,
                   param_type=6 if is_int else 9)


def px4_odometry(stamp=1_000_000, reset=0):
    return Message(kind='ODOMETRY', time_usec=stamp, reset_counter=reset,
                   frame_id=1, child_frame_id=1, estimator_type=8,
                   x=1.68, y=2.23, z=-.7, vx=.1, vy=.2, vz=0.,
                   q=[1., 0., 0., 0.], pose_covariance=[.01]*21,
                   velocity_covariance=[.02]*21)


def make_observer(mode='send', **runtime_overrides):
    host = [10_000_000_000]
    now = lambda: host[0]
    clock = GazeboSimulationClock(host_now_ns=now)
    settings = SITLOdometrySettings(enabled=True, alignment_confirmed=True,
        mount_confirmed=True, height_confirmed=True, time_mapping_confirmed=True,
        px4_reference_confirmed=True, sender_clock_domain='gazebo_sim_us',
        tag_offset_body_flu_m=(0., 0., .3), px4_reference_offset_body_flu_m=(.14, 0., 0.))
    runtime = SITLObserverSettings(**dict(dict(uncertainty_reviewed=True,
                                              min_timesync_replies=2), **runtime_overrides))
    link = None if mode == 'shadow' else Connection()
    observer = SITLObserver(settings, runtime, clock, mode=mode, connection=link,
                            dialect=DIALECT, host_now_ns=now)
    if link is not None:
        link.messages.append(Message(kind='HEARTBEAT', autopilot=12, type=2))
        link.messages.extend(parameter(name, value) for name, value in PARAMS.items())
        link.messages.append(px4_odometry())
        clock.update(900_000_000)
        link.messages.append(Message(kind='TIMESYNC', tc1=0, ts1=900_000_000))
        observer.poll_link()
        clock.update(1_000_000_000)
        link.messages.append(Message(kind='TIMESYNC', tc1=0, ts1=1_000_000_000))
        observer.poll_link()
    return observer, link, host, clock


def result(seq=0, stamp=1_000_000):
    config = configuration()
    profile, tof, imu = sensor_rows(stamp, 1.)
    processor = GazeboLiveShadow(config, profile)
    processor.add_sensor(tof)
    processor.add_sensor(imu)
    return processor.process_cycle(cycle(config, seq, stamp, [2.09, 1.68, 1.]))


def odometry_calls(link):
    return [args for kind, args in link.mav.calls if kind == 'odometry']


def test_ranges_tof_attitude_to_ned_packet_once_with_original_measurement_time():
    observer, link, host, clock = make_observer()
    row = result()
    event = observer.observe(row, raw_received_host_ns=host[0])
    assert event['transmitted'] and event['fusion_verified'] is False
    args = odometry_calls(link)[0]
    assert args[0] == row['time_us']
    np.testing.assert_allclose(args[3:6], [1.68, 2.23, -.7], atol=1e-7)
    assert all(math.isnan(v) for v in args[6])
    assert event['packet']['q'] == [None]*4
    json.dumps(log_value(event), allow_nan=False)
    assert observer.observe(row, raw_received_host_ns=host[0])['reason'] == 'duplicate_observation'
    assert len(odometry_calls(link)) == 1


def test_single_receiver_delivers_each_raw_message_once_after_observer_processing():
    observer, link, _, _ = make_observer()
    received = []
    observer.message_listener = lambda msg: received.append((msg, observer.fault))
    # The session also needs parameters the observation adapter does not use.
    extra = parameter('NAV_DLL_ACT', 3)
    reset = px4_odometry(stamp=1_040_000, reset=1)
    link.messages.extend((extra, reset))
    observer.poll_link()
    assert received == [(extra, None), (reset, 'px4_reference_reset')]
    observer.poll_link()
    assert len(received) == 2


@pytest.mark.parametrize('mode', ['shadow', 'timesync'])
def test_recording_modes_never_send_observations_even_with_confirmed_settings(mode):
    observer, link, host, _ = make_observer(mode)
    event = observer.observe(result(), raw_received_host_ns=host[0])
    assert event['candidate'] is not None and not event['transmitted']
    if link is not None:
        assert not odometry_calls(link)
        assert {name for name, _ in link.mav.calls} == {'parameter_read', 'timesync'}


@pytest.mark.parametrize('gate, expected', [
    ('settings', 'alignment_confirmed_required'),
    ('uncertainty', 'declared_uncertainty_review_required'),
    ('params', 'require_horizontal_only_ev_ctrl_1'),
    ('stale_param', 'fc_parameters_unavailable_or_stale'),
    ('warmup', 'timesync_warmup_or_fresh_reply_required'),
    ('odom', 'px4_reset_monitor_unavailable_or_stale'),
    ('wall', 'observation_wall_residence_exceeded'),
    ('sim_age', 'sensor_sample_unavailable_or_stale'),
])
def test_unverified_or_stale_inputs_cannot_reach_transport(gate, expected):
    observer, link, host, clock = make_observer()
    receipt = host[0]
    if gate == 'settings':
        observer.settings = replace(observer.settings, alignment_confirmed=False)
    elif gate == 'uncertainty':
        observer.runtime = replace(observer.runtime, uncertainty_reviewed=False)
    elif gate == 'params':
        observer.handle_message(parameter('EKF2_EV_CTRL', 3))
    elif gate == 'stale_param':
        observer.param_receipts['EKF2_EV_POS_X'] -= 6_000_000_000
    elif gate == 'warmup':
        observer.timesync_replies = 1
    elif gate == 'odom':
        observer.odometry_ns -= 600_000_000
    elif gate == 'wall':
        receipt -= 151_000_000
    elif gate == 'sim_age':
        clock.update(1_151_000_000)
    event = observer.observe(result(), raw_received_host_ns=receipt)
    assert event['reason'] == expected
    assert not odometry_calls(link)


def test_reference_shift_uses_raw_time_attitude_separately_from_tof_projection():
    config = configuration()
    profile, tof, imu = sensor_rows(1_000_000, 1.)
    processor = GazeboLiveShadow(config, profile)
    processor.add_sensor(tof)
    processor.add_sensor(imu)
    processor.add_sensor(dict(imu, time_us=1_040_000,
                             quaternion_wxyz=[math.sqrt(.5), 0., 0., math.sqrt(.5)]))
    # A future callback cannot change the attitude chosen at RAW time.
    processor.add_sensor(dict(imu, time_us=1_060_000))
    row = processor.process_cycle(cycle(config, 0, 1_040_000, [2.09, 1.68, 1.]))
    assert row['height_selection']['attitude_time_us'] == 1_000_000
    assert row['attitude_selection']['time_us'] == 1_040_000
    observer, link, host, clock = make_observer()
    clock.update(1_040_000_000)
    event = observer.observe(row, raw_received_host_ns=host[0])
    assert event['transmitted']
    np.testing.assert_allclose(odometry_calls(link)[0][3:6], [1.82, 2.09, -.7], atol=1e-7)


def test_bad_new_attitude_and_missing_tof_reject_without_reusing_previous_candidate():
    observer, link, host, clock = make_observer()
    first = result()
    assert observer.observe(first, raw_received_host_ns=host[0])['transmitted']
    for seq, mutation in enumerate((dict(height_m=None),
                                   dict(attitude_selection={'reason': 'attitude_invalid'})), 1):
        stamp = 1_000_000+seq*40_000
        clock.update(stamp*1000)
        row = dict(result(seq, stamp), **mutation)
        assert not observer.observe(row, raw_received_host_ns=host[0])['transmitted']
    assert len(odometry_calls(link)) == 1


@pytest.mark.parametrize('message, reason', [
    (px4_odometry(1_040_000, 1), 'px4_reference_reset'),
    (px4_odometry(900_000), 'px4_time_reversed'),
    (Message(kind='TIMESYNC', tc1=0, ts1=500_000_000), 'px4_request_time_reversed'),
])
def test_px4_reset_or_clock_reversal_latches_until_new_run(message, reason):
    observer, link, host, clock = make_observer()
    observer.handle_message(message)
    assert observer.fault == reason
    observer.handle_message(px4_odometry(1_100_000))
    assert observer.observe(result(), raw_received_host_ns=host[0])['reason'] == reason
    assert not odometry_calls(link)


def test_clock_stall_blocks_then_new_tick_recovers_but_reversal_latches():
    observer, link, host, clock = make_observer()
    host[0] += 260_000_000
    assert observer.observe(result(), raw_received_host_ns=host[0])['reason'] == 'simulation_clock_stalled'
    clock.update(1_040_000_000)
    assert observer.observe(result(1, 1_040_000), raw_received_host_ns=host[0])['transmitted']
    with pytest.raises(ClockUnavailable):
        clock.update(1_000_000_000)
    assert observer.observe(result(2, 1_080_000), raw_received_host_ns=host[0])['reason'] == 'simulation_time_reversed'
    assert observer.fault == 'simulation_time_reversed'


def test_b_uses_its_real_window_and_rejects_an_old_prediction():
    observer, link, host, clock = make_observer(model='B')
    config = configuration()
    profile, _, _ = sensor_rows(1_000_000, 1.)
    processor = GazeboLiveShadow(config, profile)
    for seq in range(4):
        stamp = 1_000_000+seq*40_000
        _, tof, imu = sensor_rows(stamp, 1.)
        processor.add_sensor(tof)
        processor.add_sensor(imu)
        row = processor.process_cycle(cycle(config, seq, stamp, [2.09, 1.68, 1.]))
        clock.update(stamp*1000)
        event = observer.observe(row, raw_received_host_ns=host[0])
        assert event['transmitted'] == row['models']['B']['ok']
    assert event['transmitted']
    count = len(odometry_calls(link))
    row.update(seq=4, time_us=1_160_000)
    row['models']['B']['fresh_observation_count'] = 0
    event = observer.observe(row, raw_received_host_ns=host[0])
    assert event['reason'] == 'new_range_observation_required'
    assert len(odometry_calls(link)) == count


def test_link_backlog_prevents_sending_ahead_of_unprocessed_reset():
    observer, link, host, clock = make_observer()
    link.messages.extend([Message(kind='HEARTBEAT', autopilot=12, type=2),
                          px4_odometry(1_100_000, 1)])
    events = observer.poll_link(max_messages=1)
    assert events[-1]['type'] == 'receive_backlog'
    assert observer.observe(result(), raw_received_host_ns=host[0])['reason'] == 'mavlink_queue_not_drained'
    observer.poll_link()
    assert observer.fault == 'px4_reference_reset'
    assert not odometry_calls(link)


def test_transport_failure_latches_and_parameter_reads_never_change_values():
    observer, link, host, _ = make_observer()
    read_names = {args[2].decode() for name, args in link.mav.calls if name == 'parameter_read'}
    assert read_names == set(PARAMETERS)
    def fail(*args):
        raise OSError('injected_send_failure')
    link.mav.odometry_send = fail
    with pytest.raises(OSError, match='injected_send_failure'):
        observer.observe(result(), raw_received_host_ns=host[0])
    assert observer.fault == 'odometry_send_failed'
    assert not observer.observe(result(1, 1_040_000), raw_received_host_ns=host[0])['transmitted']


def test_px4_position_velocity_frames_and_mode_are_retained_as_native_evidence():
    observer, link, host, clock = make_observer()
    message = px4_odometry(1_040_000)
    message.child_frame_id = 12
    message.vz = float('nan')
    event = observer.handle_message(message)
    assert event['payload']['x'] == 1.68 and event['payload']['vy'] == .2
    assert event['payload']['child_frame_id'] == 12
    assert event['payload']['time_usec'] == 1_040_000
    assert event['payload']['vz'] is None and math.isnan(message.vz)
    assert event['fusion_verified'] is False and event['flight_valid'] is False
    json.dumps(event, allow_nan=False)
    event = observer.handle_message(Message(kind='HEARTBEAT', autopilot=12, type=2,
                                            base_mode=129, custom_mode=393216, system_status=4))
    assert event['payload']['base_mode'] == 129
    assert event['payload']['custom_mode'] == 393216


@pytest.mark.parametrize('kind, fields', [
    ('LOCAL_POSITION_NED', dict(time_boot_ms=1000, x=2., y=3., z=-1., vx=.1, vy=.2, vz=0.)),
    ('POSITION_TARGET_LOCAL_NED', dict(time_boot_ms=1000, coordinate_frame=1,
                                     type_mask=0, x=4., y=5., z=-1., vz=float('nan'))),
    ('ESTIMATOR_STATUS', dict(time_usec=1_000_000, flags=15, pos_horiz_ratio=.3)),
    ('ESTIMATOR_SENSOR_FUSION_STATUS', dict(time_usec=1_000_000, arbitrary_future_field=7)),
    ('EXTENDED_SYS_STATE', dict(landed_state=1, vtol_state=0)),
    ('GPS_GLOBAL_ORIGIN', dict(time_usec=3224000, latitude=473979860, longitude=85461920, altitude=200)),
    ('POSITION_TARGET_GLOBAL_INT', dict(time_boot_ms=1000, coordinate_frame=5,
                                       type_mask=0, lat_int=473979960, lon_int=85462020)),
    ('COMMAND_ACK', dict(command=192, result=0, target_system=245, target_component=191)),
])
def test_telemetry_keeps_payload_without_changing_model_parameters_or_sending(kind, fields):
    observer, link, host, clock = make_observer()
    before_calls, before_params = list(link.mav.calls), dict(observer.params)
    event = observer.handle_message(Message(kind=kind, **fields))
    assert event['payload'] == log_value(dict(mavpackettype=kind, **fields))
    assert observer.params == before_params and link.mav.calls == before_calls
    assert observer.handle_message(Message(kind=kind, source_system=2, **fields)) is None
