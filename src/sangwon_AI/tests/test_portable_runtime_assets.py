"""Runtime bytes must survive Windows checkout and Linux execution unchanged."""
import hashlib
import io
import json
from pathlib import Path
import subprocess
import tarfile


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


def test_windows_archive_keeps_recorded_bytes_and_linux_runtime_bytes(tmp_path):
    def git(*args):
        return subprocess.check_output(['git',*args],cwd=tmp_path,stderr=subprocess.DEVNULL)
    git('init','-q','-b','asset-test')
    git('config','core.autocrlf','true')
    (tmp_path/'.gitattributes').write_bytes((ROOT.parents[1]/'.gitattributes').read_bytes())
    recorded = b'{"recorded":true}\r\n'
    script = b'#!/bin/bash\r\nset -eo pipefail\r\n'
    paths = {'data/raw/test.jsonl':recorded, 'src/sangwon_AI/tests/test.sh':script}
    for name, content in paths.items():
        p=tmp_path/name; p.parent.mkdir(parents=True,exist_ok=True); p.write_bytes(content)
    git('add','.')
    git('-c','user.name=Asset Test','-c','user.email=fixture@example.invalid',
        'commit','-q','-m','Synthetic portable asset fixture')
    with tarfile.open(fileobj=io.BytesIO(git('archive','--format=tar','HEAD'))) as archive:
        assert archive.extractfile('data/raw/test.jsonl').read() == recorded
        assert archive.extractfile('src/sangwon_AI/tests/test.sh').read() == script.replace(b'\r\n',b'\n')
