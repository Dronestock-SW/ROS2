"""Adapt the selected UWB sender status without inventing fusion evidence."""
import math

# Historical default; Settings.bridge_observation_timeout_s is the live limit.
BRIDGE_OBSERVATION_MAX_AGE_S = .35


def normalize_bridge(value, settings):
    if settings.bridge_topic == '/uwb/bridge_status':
        return value
    cfg = value.get('settings', {})
    counts = value.get('counts', {})
    if not isinstance(cfg, dict) or not isinstance(counts, dict):
        return dict(gate='local_odometry_status_unavailable', settings={})
    count = counts.get('published')
    stamp = value.get('last_sample_stamp_ns')
    age = value.get('last_send_age_s')
    if (not settings.bridge_host_boot_id
            or type(count) is not int or count < 0
            or type(stamp) is not int or stamp <= 0
            or type(age) not in (int, float) or not math.isfinite(age)):
        return dict(gate='local_odometry_status_unavailable', settings={})
    result = dict(value, published=count, last_observation_stamp_ns=stamp,
                  settings=dict(cfg, input_source='local_odometry'))
    if cfg.get('host_boot_id') != settings.bridge_host_boot_id:
        result.update(gate='local_odometry_status_unavailable', bridge_contract_error='bridge_host_boot_changed')
    elif not 0 <= age <= settings.bridge_observation_timeout_s or value.get('timesync_ready') is not True:
        result['gate'] = 'local_odometry_status_unavailable'
    return result
