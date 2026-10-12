"""Normal PX4 ARM request eligibility; never force-arm or change flight mode."""

from datetime import datetime
from .contracts import finite


def arm_blocker(settings, sample, *, idle, recording_ok):
    if not settings.execute:
        return 'execution_disabled'
    if not idle:
        return 'mission_or_arm_request_in_progress'
    if not recording_ok:
        return 'command_recording_unavailable'
    if not sample.connected or not 0 <= sample.state_age_s <= 1.:
        return 'fc_disconnected_or_stale'
    if sample.armed or sample.landed != 1 or not 0 <= sample.landed_age_s <= 1.:
        return 'fresh_disarmed_ground_required'
    # ARM in AUTO.TAKEOFF can start climbing immediately. Manual ARM stays
    # in the pilot's existing manual mode and never schedules a later START.
    if sample.mode not in ('POSCTL', 'ALTCTL'):
        return 'manual_position_or_altitude_mode_required'
    if not sample.rc_valid or not 0 <= sample.rc_age_s <= 1. or not sample.rc_mapping_valid:
        return 'fresh_physical_rc_required'
    if not finite(sample.battery) or not .3 <= sample.battery <= 1. or not 0 <= sample.battery_age_s <= 3.:
        return 'battery_unavailable_or_low'
    if not sample.command_services_ready:
        return 'mavros_command_services_unavailable'
    return ''


def arm_request(payload, settings, session_id, wall_s, *, action='arm'):
    if (payload.get('ok') is not True or payload.get('contract_version') != '1.0'
            or payload.get('control_action') != action
            or str(payload.get('drone_id')) != settings.drone_id
            or payload.get('anchor_layout_id') != settings.layout_id
            or payload.get('operator_session_id') != session_id):
        raise ValueError('arm_device_or_session_mismatch')
    key = payload.get('control_request_id')
    if not isinstance(key, str) or not 8 <= len(key) <= 128:
        raise ValueError('arm_request_id_required')
    try:
        stamp = datetime.fromisoformat(payload['control_requested_at'].replace('Z', '+00:00'))
        valid = stamp.tzinfo is not None and 0 <= wall_s-stamp.timestamp() <= 3.
    except (KeyError, TypeError, AttributeError, ValueError):
        valid = False
    if not valid:
        raise ValueError('arm_request_expired_or_future')
    return key
