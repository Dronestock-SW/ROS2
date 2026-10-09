from dataclasses import replace
from drone_mission.contracts import Settings
from drone_mission.preflight import field_preflight
from drone_mission.session import Snapshot


def test_reports_all_current_field_conflicts_without_enabling_anything():
    settings = Settings(full_mission=True, drone_id='6', expected_mis_takeoff_alt_m=1.3)
    sample = Snapshot(takeoff_alt_m=1.3, takeoff_param_age_s=0, mag_type=0, rc_mode=3, rc_override=2)
    report = field_preflight(settings, sample, map_loaded=False, recording_ok=True)
    assert {'execution', 'heading_policy', 'rc_mode', 'rc_auto_override', 'origin', 'rc', 'battery', 'uwb'} <= set(report['blockers'])
    assert 'height_parameter' not in report['blockers']
    assert not settings.execute and not settings.fusion_confirmed
    assert report['scope'] == 'observation_only_not_flight_authorization'


def test_unavailable_or_stale_parameters_are_not_matching_values():
    settings = Settings(expected_mis_takeoff_alt_m=1.7)
    s = Snapshot(takeoff_alt_m=1.7, takeoff_param_age_s=6, mag_type=0, takeoff_action=0, rc_mode=0, rc_override=3)
    r = field_preflight(settings, s, map_loaded=False, recording_ok=False)
    assert {'height_parameter', 'takeoff_action', 'heading_policy', 'rc_mode', 'rc_auto_override'} <= set(r['blockers'])
    r = field_preflight(settings, replace(s, takeoff_param_age_s=0), map_loaded=False, recording_ok=False)
    assert not {'height_parameter', 'takeoff_action', 'heading_policy', 'rc_mode', 'rc_auto_override'} & set(r['blockers'])


def test_const_zero_default_snapshot_cannot_appear_ready():
    r = field_preflight(Settings(), Snapshot(), map_loaded=False, recording_ok=False)
    assert not r['checked_inputs_passed'] and len(r['blockers']) > 20
