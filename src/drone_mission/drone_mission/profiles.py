"""Resolve one Tag, domain and measured layout before starting any process."""
import json
from pathlib import Path

from drone_uwb.integration.ros.layout_selection import matched_anchor_path
from .contracts import Settings


def trial_profile(tag, mission_share, uwb_share, config='', btf_config='', anchor_file=''):
    role = tag.lower()
    if role not in ('a', 'b'):
        raise ValueError('tag must be A or B')
    tag_id, domain = ('5', 1) if role == 'a' else ('6', 2)
    mission_share, uwb_share = Path(mission_share), Path(uwb_share)
    config = Path(config) if config else mission_share / 'config' / (
        'flight.json' if role == 'a' else 'flight_tag_b.json')
    settings = Settings(**json.loads(config.read_text(encoding='utf-8')))
    btf_config = Path(btf_config) if btf_config else uwb_share / f'config/runtime/uwb_btf_tag_{role}.json'
    measured = json.loads(btf_config.read_text(encoding='utf-8'))
    if (settings.drone_id != tag_id or measured['tag_id'] != tag_id
            or measured.get('tdma_mode') != 'required'):
        raise ValueError('mission and BTF must match selected Tag with tdma_mode=required')
    if settings.layout_id != measured['layout_id']:
        raise ValueError('mission and BTF layout must match')
    anchors = matched_anchor_path(uwb_share, measured, anchor_file)
    bridge = {name: getattr(settings, name) for name in (
        'layout_confirmed', 'alignment_confirmed', 'timing_confirmed', 'sensor_mount_confirmed',
        'enu_yaw_deg', 'enu_offset_x_m', 'enu_offset_y_m', 'expected_ev_delay_ms')}
    bridge.update(input_source='btf_xy', tag_id=tag_id)
    bridge.update({f'antenna_body_frd_{axis}_m': getattr(settings, f'expected_ev_pos_{axis}_m')
                   for axis in ('x', 'y', 'z')})
    return settings, domain, config, btf_config, anchors, bridge
