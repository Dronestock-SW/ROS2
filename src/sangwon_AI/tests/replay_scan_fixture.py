"""Explicit synthetic map/scan bundle. Never an operational mission or calibration."""
import hashlib
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]


def bundle(drone='TEST-DRONE-01', outcome='SCANNER_SUCCESS'):
    directory = ROOT / 'contracts/web_v1_1_draft4'
    snapshot = json.loads((directory / 'snapshot_content.json').read_bytes())
    snapshot.update(drone_id=drone, snapshot_id='synthetic-scan-' + drone)
    # Long validity is permitted for this immutable synthetic fixture only.
    # Exact snapshot and profile byte hashes still gate every preparation.
    for approval in snapshot['approvals']:
        approval['approved_at'] = '2026-10-04T00:00:00Z'
        approval['valid_until'] = '2099-01-01T00:00:00Z'
    artifacts = {}
    for name in ('motion-demo', 'body-demo', 'scan-demo'):
        text = (directory / (name + '.json')).read_bytes().replace(b'\r\n', b'\n').decode('utf-8')
        value = json.loads(text)
        artifacts[name + '@' + value['revision']] = text
        for ref in snapshot['profile_artifacts']:
            if ref['id'] == name:
                ref['sha256'] = hashlib.sha256(text.encode()).hexdigest()
    raw = json.dumps(snapshot, sort_keys=True, separators=(',', ':')).encode()
    config = {'replay_profile_artifacts': artifacts,
        'approved_replay_launch_pose': {'position_m': {'x': 1.5, 'y': 1.5, 'z': 0}, 'yaw_deg': 0},
        'replay_scan_source': 'SYNTHETIC_SCAN_FIXTURE', 'replay_scan_outcomes': {'S01': outcome},
        'approved_replay_snapshot_sha256': [hashlib.sha256(raw).hexdigest()]}
    return raw, config
