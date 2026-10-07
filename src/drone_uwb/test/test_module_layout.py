"""Check relocation integrity and the receive/processing ownership boundary."""
import ast
import hashlib
import importlib
import json
from pathlib import Path

import pytest

from drone_uwb.integration.recording import open_record_files

ROOT = Path(__file__).resolve().parents[3]
PACKAGE = ROOT / 'src/drone_uwb/drone_uwb'


def test_catalog_bytes_and_historical_paths():
    catalog = json.loads((ROOT / 'data/catalog.json').read_text(encoding='utf-8'))
    for item in catalog['files']:
        path = ROOT / item['path']
        assert hashlib.sha256(path.read_bytes()).hexdigest() == item['sha256']
        assert path.stat().st_size == item['bytes']
        assert (ROOT / item['legacy_path']).resolve() == path.resolve()


def test_legacy_imports_share_implementation_and_exception_identity():
    mapping = json.loads((ROOT / 'data/module_paths.json').read_text(encoding='utf-8'))
    for old, new in mapping.items():
        if old in ('drone_uwb.node', 'drone_uwb.bridge', 'drone_uwb.bench_probe'):
            continue  # ROS imports are checked after the colcon build.
        assert importlib.import_module(old) is importlib.import_module(new)


def test_lower_layers_do_not_import_ros_or_integration():
    for layer in ('contracts', 'acquisition', 'processing'):
        for path in (PACKAGE / layer).rglob('*.py'):
            for node in ast.walk(ast.parse(path.read_text(encoding='utf-8'))):
                if isinstance(node, ast.Import):
                    names = [alias.name for alias in node.names]
                elif isinstance(node, ast.ImportFrom):
                    assert node.level == 0, path
                    names = [node.module or '']
                else:
                    continue
                forbidden = ['rclpy', 'geometry_msgs', 'mavros_msgs', 'drone_uwb.integration', 'drone_uwb.preimu']
                if layer in ('contracts', 'acquisition'):
                    forbidden.append('drone_uwb.processing')
                if layer == 'contracts':
                    forbidden.append('drone_uwb.acquisition')
                assert not any(name == prefix or name.startswith(prefix + '.')
                               for name in names for prefix in forbidden), path


def test_recording_separates_bytes_and_decisions_and_refuses_reuse(tmp_path):
    target = tmp_path / 'session'
    files = open_record_files(target, {'purpose': '수신 시험'})
    try:
        files['raw'].write(b'broken\xff\n')
        files['received'].write('{"message": {}}\n')
        files['decisions'].write('{"reason": "rejected"}\n')
    finally:
        for stream in files.values():
            stream.close()
    assert (target / 'raw/serial.raw').read_bytes() == b'broken\xff\n'
    assert (target / 'processed/decisions.jsonl').read_text() == '{"reason": "rejected"}\n'
    with pytest.raises(FileExistsError):
        open_record_files(target, {})
    assert (target / 'raw/serial.raw').read_bytes() == b'broken\xff\n'
