"""Target framing, precision and missing-evidence guards for SITL commands."""
from dataclasses import replace
import math
from types import SimpleNamespace

import pytest

from drone_uwb.integration.sitl.sitl_odometry_contract import SITLOdometrySettings
from drone_uwb.integration.sitl.sitl_target_contract import (
    EARTH_RADIUS_M, NavigationReadiness, PX4GlobalReference, SITLTargetSettings,
    global_to_ned, ned_to_global, reposition_fields, send_reposition_fields)


ALIGNMENT = SITLOdometrySettings(alignment_confirmed=True, px4_reference_confirmed=True)
SETTINGS = SITLTargetSettings(enabled=True, sitl_session_confirmed=True, observation_fusion_confirmed=True)
REFERENCE = PX4GlobalReference(47.397986, 8.546192, 3_224_000)
STATE = NavigationReadiness(.02, .02, 3_224_000, 0, True, True, True, True, True, True, True)


def fields(settings=SETTINGS, alignment=ALIGNMENT, state=STATE, target=(2.59, 1.68)):
    return reposition_fields(target, (2.09, 1.68), settings, alignment,
                             REFERENCE, state, expected_reset_counter=0)


def test_equatorial_cardinal_axes_have_analytic_distances():
    origin = PX4GlobalReference(0., 0., 1)
    latitude, longitude = ned_to_global(1000., 0., origin)
    assert latitude == pytest.approx(math.degrees(1000./EARTH_RADIUS_M), abs=1e-12)
    assert longitude == 0.
    latitude, longitude = ned_to_global(0., 1000., origin)
    assert latitude == pytest.approx(0., abs=1e-12)
    assert longitude == pytest.approx(math.degrees(1000./EARTH_RADIUS_M), abs=1e-12)


@pytest.mark.parametrize('origin', [REFERENCE, PX4GlobalReference(-33., 151., 1),
                                   PX4GlobalReference(0., 179.99999, 1)])
@pytest.mark.parametrize('point', [(0., 0.), (1.68, 2.09), (-7., -5.), (100., 20.)])
def test_projection_round_trip_and_longitude_wrap(origin, point):
    lat, lon = ned_to_global(*point, origin)
    assert -180 <= lon <= 180
    assert global_to_ned(lat, lon, origin) == pytest.approx(point, abs=1e-7)


def test_target_uses_shared_map_transform_and_keeps_altitude_unspecified():
    result = fields()
    assert result['target_ned_xy_m'] == [1.68, 2.59]
    assert result['target_quantization_error_m'] < .008
    assert result['reference_point'] == 'px4_reference'
    assert math.isnan(result['z']) and math.isnan(result['param4'])
    assert not result['arrival_valid'] and not result['target_applied']
    rotated = fields(alignment=replace(ALIGNMENT, enu_yaw_deg=90., enu_offset_x_m=.2,
                                       enu_offset_y_m=-.3))
    assert rotated['target_ned_xy_m'] == pytest.approx([2.29, -1.48])


@pytest.mark.parametrize('name', ['enabled', 'sitl_session_confirmed', 'observation_fusion_confirmed'])
def test_trial_confirmations_are_required(name):
    with pytest.raises(ValueError, match=name):
        fields(settings=replace(SETTINGS, **{name: False}))


@pytest.mark.parametrize('name', ['armed', 'in_air', 'hold_mode', 'xy_valid', 'vxy_valid',
                                 'attitude_solution_valid', 'global_position_valid'])
def test_fc_state_must_be_verified_for_each_command(name):
    with pytest.raises(ValueError, match=name):
        fields(state=replace(STATE, **{name: False}))


@pytest.mark.parametrize('changes,reason', [
    ({'state_age_s': .201}, 'stale_or_future'), ({'uwb_age_s': .251}, 'stale_or_future'),
    ({'state_age_s': -.01}, 'stale_or_future'), ({'reset_counter': 1}, 'reference_reset'),
    ({'origin_timestamp_us': 3_300_000}, 'global_reference_changed'),
])
def test_stale_input_and_reference_changes_stop_command(changes, reason):
    with pytest.raises(ValueError, match=reason):
        fields(state=replace(STATE, **changes))


@pytest.mark.parametrize('target,reason', [((0., 0.), 'outside_trial'), ((4., 2.), 'leg_exceeds'),
                                         ((float('nan'), 2.), 'finite_map')])
def test_target_limits_are_applied_before_any_wire_output(target, reason):
    with pytest.raises(ValueError, match=reason):
        fields(target=target)


def test_transport_handoff_is_once_and_does_not_claim_arrival():
    calls = []
    sender = SimpleNamespace(command_int_send=lambda *args: calls.append(args))
    dialect = SimpleNamespace(MAV_FRAME_GLOBAL=0, MAV_CMD_DO_REPOSITION=192)
    packet = fields()
    send_reposition_fields(sender, dialect, packet)
    assert calls[0][:6] == (1, 1, 0, 192, 0, 0)
    assert calls[0][6:8] == (.3, 0.)  # no automatic mode change
    assert all(math.isnan(calls[0][i]) for i in (8, 9, 12))
    assert packet['transmitted'] and not packet['target_applied'] and not packet['arrival_valid']
    with pytest.raises(ValueError, match='repeated'):
        send_reposition_fields(sender, dialect, packet)
    assert len(calls) == 1


def test_ambiguous_transport_failure_is_not_automatically_retried():
    def fail(*args):
        raise OSError('unknown handoff state')
    packet = fields()
    sender = SimpleNamespace(command_int_send=fail)
    dialect = SimpleNamespace(MAV_FRAME_GLOBAL=0, MAV_CMD_DO_REPOSITION=192)
    with pytest.raises(OSError):
        send_reposition_fields(sender, dialect, packet)
    assert not packet['sender_handoff_ready']
    with pytest.raises(ValueError):
        send_reposition_fields(sender, dialect, packet)
