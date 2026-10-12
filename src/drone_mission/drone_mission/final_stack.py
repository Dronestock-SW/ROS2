"""Static FINAL launch/alignment contracts. No ROS, FC or process operations."""
from copy import deepcopy
import importlib
import math

from drone_uwb.integration.local_odometry import LocalObservationSettings, alignment_yaw_deg
from .contracts import Settings


def validate_field_configuration(mission, alignment, *, host_boot_id):
    cfg = Settings(**mission)
    obs = LocalObservationSettings(**alignment['settings'])
    if not cfg.local_xy or not cfg.anchor_trial or not cfg.execute or not obs.enabled:
        raise ValueError('explicit_local_xy_anchor_trial_required')
    if cfg.mission_ai != 'cpp_bt':
        raise ValueError('final_field_requires_cpp_bt_mission_ai')
    if cfg.bridge_topic != '/uwb/local_odometry_status':
        raise ValueError('local_odometry_status_required')
    if obs.host_boot_id != host_boot_id or cfg.bridge_host_boot_id != host_boot_id:
        raise ValueError('host_reboot_requires_new_alignment_evidence')
    if cfg.drone_id != obs.tag_id:
        raise ValueError('mission_observation_tag_mismatch')
    if (obs.alignment_method != 'operator_cardinal_and_stationary_px4_yaw'
            or type(obs.fc_remote_ns_at_alignment) is not int
            or obs.fc_remote_ns_at_alignment <= 0):
        raise ValueError('alignment_evidence_required')
    if not all((cfg.layout_confirmed, cfg.sensor_mount_confirmed,
                obs.layout_confirmed, obs.sensor_mount_confirmed)):
        raise ValueError('layout_or_mount_unconfirmed')
    for key in ('enu_yaw_deg', 'map_y_axis_sign', 'enu_offset_x_m', 'enu_offset_y_m'):
        if abs(getattr(cfg, key)-getattr(obs, key)) > 1e-6:
            raise ValueError('observation_mission_transform_mismatch:' + key)
    for expected, actual in (
            ('expected_ev_delay_ms', 'expected_ev_delay_ms'),
            ('expected_ev_pos_x_m', 'antenna_body_frd_x_m'),
            ('expected_ev_pos_y_m', 'antenna_body_frd_y_m'),
            ('expected_ev_pos_z_m', 'antenna_body_frd_z_m')):
        if abs(getattr(cfg, expected)-getattr(obs, actual)) > 1e-6:
            raise ValueError('observation_mission_sensor_contract_mismatch:' + expected)
    return cfg, obs


def stationary_yaw_mean(samples):
    if len(samples) < 50 or any(type(v) not in (int, float) or not math.isfinite(v) for v in samples):
        raise ValueError('fifty_finite_yaw_samples_required')
    mean = math.degrees(math.atan2(
        sum(math.sin(math.radians(v)) for v in samples),
        sum(math.cos(math.radians(v)) for v in samples)))
    relative = [math.remainder(v-mean, 360.) for v in samples]
    span = max(relative)-min(relative)
    if span > 1.:
        raise ValueError('stationary_yaw_span_exceeded')
    return mean, span


def aligned_field_configuration(mission, alignment, *, body_enu_yaw_deg,
                                host_boot_id, fc_remote_ns, operator_heading="-Y"):
    mission, alignment = deepcopy(mission), deepcopy(alignment)
    if operator_heading not in ('-Y', '+Y'):
        raise ValueError('operator_heading_must_be_plus_or_minus_y')
    angle = alignment_yaw_deg(body_enu_yaw_deg, 90. if operator_heading == '+Y' else -90., alignment['settings']['map_y_axis_sign'])
    alignment['settings'].update(host_boot_id=host_boot_id,
        fc_remote_ns_at_alignment=fc_remote_ns, enu_yaw_deg=angle)
    mission.update(bridge_host_boot_id=host_boot_id, enu_yaw_deg=angle)
    validate_field_configuration(mission, alignment, host_boot_id=host_boot_id)
    return mission, alignment


def require_field_dependencies():
    # Import only: no node creation, subscriptions, sockets or FC commands.
    for module in ('rclpy', 'mavros.mavlink', 'pymavlink', 'websockets.asyncio.server',
                   'drone_mission.node', 'drone_platform_link.runtime',
                   'drone_uwb.integration.ros.local_odometry_bridge'):
        try:
            importlib.import_module(module)
        except ImportError as exc:
            raise RuntimeError('field_runtime_dependency_unavailable:' + module) from exc
