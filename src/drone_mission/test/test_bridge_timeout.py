"""Configured delivery timeout never extends raw UWB or FC pose freshness."""
import math
from dataclasses import replace
import pytest
from drone_mission.contracts import Settings
from drone_mission.bridge_status import normalize_bridge
from drone_mission.session import FlightSession
from drone_mission.preflight import field_preflight
from test_session import settings, snapshot
from test_anchor_trial import config, inputs


@pytest.mark.parametrize('limit,age,ready', [
    (.35,.48,False), (.7,.48,True), (.7,.7,True), (.7,.700001,False),
])
def test_delivery_age_at_requested_boundary(limit, age, ready):
    cfg = settings(bridge_observation_timeout_s=limit)
    session = FlightSession(cfg)
    session.last_web_s = 10.
    reason = session.ready(snapshot(uwb_age_s=.13, bridge_observation_age_s=age), 10.)
    assert (reason == 'ready') is ready
    if not ready:
        assert reason == 'uwb_px4_bridge_unavailable'


@pytest.mark.parametrize('changes,reason', [
    ({'uwb_age_s':.250001}, 'uwb_unavailable'),
    ({'pose_age_s':.200001}, 'px4_pose_unavailable_or_outside_bounds'),
    ({'estimator_valid':False}, 'px4_estimator_unavailable'),
    ({'bridge_ready':False}, 'uwb_px4_bridge_unavailable'),
])
def test_longer_delivery_limit_preserves_independent_guards(changes, reason):
    sample = snapshot(uwb_age_s=.13, bridge_observation_age_s=.48)
    assert FlightSession(settings(bridge_observation_timeout_s=.7)).ready(
        replace(sample, **changes), 10.) == reason


@pytest.mark.parametrize('age,ready', [(.48,True), (.7,True), (.700001,False)])
def test_sender_normalization_obeys_same_configuration(age, ready):
    cfg = settings(bridge_topic='/uwb/local_odometry_status', bridge_host_boot_id='fixture',
                   bridge_observation_timeout_s=.7)
    value = dict(gate='ready', settings=dict(host_boot_id='fixture'), counts=dict(published=12),
                 last_sample_stamp_ns=123, last_send_age_s=age, timesync_ready=True)
    assert (normalize_bridge(value, cfg)['gate'] == 'ready') is ready


def test_preflight_and_executor_agree_about_reported_point_four_eight():
    cfg = config(bridge_observation_timeout_s=.7)
    sample = inputs(uwb_age_s=.13, bridge_observation_age_s=.48)
    report = field_preflight(cfg, sample, map_loaded=False, recording_ok=True)
    assert report['checked_inputs_passed']
    assert report['bridge_observation_timeout_s'] == .7


@pytest.mark.parametrize('value', [0,-1,.0001,.700001,True,'0.7',math.nan,math.inf])
def test_invalid_delivery_limits_are_rejected(value):
    with pytest.raises(ValueError, match='bridge_observation_timeout'):
        Settings(bridge_observation_timeout_s=value)


def test_old_configuration_keeps_default():
    assert Settings().bridge_observation_timeout_s == .35
