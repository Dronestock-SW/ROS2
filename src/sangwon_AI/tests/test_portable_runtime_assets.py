"""Runtime bytes must survive Windows checkout and Linux execution unchanged."""
import hashlib
import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_linux_runtime_scripts_have_lf_line_endings():
    scripts = list(ROOT.rglob('*.sh'))
    scripts += list((ROOT.parent/'drone_uwb/deployment').glob('*.sh'))
    assert scripts
    for path in scripts:
        assert b'\r\n' not in path.read_bytes(), str(path)


def test_approved_replay_snapshot_uses_exact_portable_bytes():
    config = json.loads((ROOT/'config/companion.replay.json').read_text(encoding='utf-8'))
    snapshot = (ROOT/'contracts/runtime_replay/waypoint_snapshot.json').read_bytes()
    assert config['profile'] == 'REPLAY' and config['physical_output_enabled'] is False
    assert hashlib.sha256(snapshot).hexdigest() in config['approved_replay_snapshot_sha256']
