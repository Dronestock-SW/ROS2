"""Runtime evidence policy. FC health guards remain separate from reports."""
import math


def recent(value, limit):
    return type(value) in (float, int) and math.isfinite(value) and 0 <= value <= limit


def observation_status(settings, sample, feedback_age_s=None):
    warnings = []
    if not recent(sample.estimator_age_s,.5):
        warnings.append('fc_estimator_report_delayed')
    pose_ready=recent(sample.pose_age_s,settings.pose_timeout_s) and len(sample.xy)==2
    if not pose_ready:
        warnings.append('fc_position_feedback_missing')
    raw_ready = recent(sample.uwb_age_s, .25) and len(sample.uwb_xy) == 2
    if not raw_ready:
        warnings.append('uwb_observation_missing')
    if not recent(sample.bridge_age_s, 2.5):
        warnings.append('bridge_report_delayed')
    if not recent(sample.bridge_observation_age_s, settings.bridge_observation_timeout_s):
        warnings.append('bridge_delivery_report_delayed')
    if not sample.bridge_ready:
        warnings.append('bridge_report_not_ready')
    if feedback_age_s is not None and not recent(feedback_age_s, .5):
        warnings.append('target_feedback_delayed')
    error = getattr(sample, 'bridge_contract_error', '')
    report_current = recent(sample.bridge_age_s,2.5) and recent(sample.bridge_observation_age_s,settings.bridge_observation_timeout_s)
    if error and not report_current:
        warnings.append('unverified_contract_report')
        error = ''
    # A decoded, ready report explicitly contradicting the coordinate contract
    # is different from missing/malformed telemetry.
    if not error and sample.bridge_ready and report_current and not sample.alignment_matches:
        error = 'observation_and_command_alignment_mismatch'
    hold = not raw_ready or not pose_ready
    return dict(policy='fc_owned_runtime', warnings=warnings, contract_error=error,
                progress_held=hold, mode='FC_HOLD' if hold else 'REPORT_DELAY' if warnings else 'NORMAL',
                raw_observation_fresh=raw_ready, land_requested_by_report=False)
