"""Protect CLI, exception and configuration behavior across module relocation."""
import importlib
import os
from pathlib import Path
import subprocess
import sys
from types import ModuleType

import pytest


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


@pytest.mark.parametrize('group,name', [
    ('gazebo', 'gazebo_capture'),
    ('gazebo', 'gazebo_sensors'),
    ('gazebo', 'gazebo_live_shadow'),
    ('gazebo', 'gazebo_ranges'),
    ('gazebo', 'gazebo_rig'),
    ('sitl', 'sitl_clock_probe'),
    ('sitl', 'sitl_link_probe'),
])
def test_legacy_and_canonical_cli_help_match(group, name):
    environment = dict(os.environ)
    environment['PYTHONPATH'] = os.pathsep.join(filter(None, [
        str(PACKAGE_ROOT), environment.get('PYTHONPATH')]))
    outputs = []
    for module in (f'drone_uwb.integration.{name}',
                   f'drone_uwb.integration.{group}.{name}'):
        result = subprocess.run(
            [sys.executable, '-m', module, '--help'],
            env=environment, capture_output=True, text=True, timeout=20,
            check=False)
        assert result.returncode == 0, result.stderr
        assert result.stderr == ''
        assert 'usage:' in result.stdout
        outputs.append(result.stdout)
    assert outputs[0] == outputs[1]


def test_legacy_clock_exception_and_monkeypatch_reach_implementation(monkeypatch):
    old = importlib.import_module('drone_uwb.integration.gazebo_clock')
    current = importlib.import_module('drone_uwb.integration.gazebo.gazebo_clock')
    timesync = importlib.import_module('drone_uwb.integration.sitl.sitl_timesync')
    assert old is current
    assert timesync.ClockUnavailable is old.ClockUnavailable
    clock = current.GazeboSimulationClock(host_now_ns=lambda: 1)
    with pytest.raises(old.ClockUnavailable, match='simulation_clock_unavailable'):
        clock.now_ns()

    class PatchedClockError(ValueError):
        pass

    monkeypatch.setattr(old, 'ClockUnavailable', PatchedClockError)
    with pytest.raises(PatchedClockError, match='simulation_clock_unavailable'):
        clock.now_ns()


def test_rig_defaults_resolve_source_configuration():
    rig = importlib.import_module('drone_uwb.integration.gazebo.gazebo_rig')
    directory = rig._default_config_directory()
    assert directory == PACKAGE_ROOT / 'config'
    for name in ('anchors/anchors_20261004.json', 'gazebo_equipment.json', 'gazebo_shadow.json'):
        assert (directory / name).is_file()


def test_rig_defaults_resolve_installed_package_share(tmp_path, monkeypatch):
    rig = importlib.import_module('drone_uwb.integration.gazebo.gazebo_rig')
    monkeypatch.setattr(rig, '__file__', str(
        tmp_path / 'lib/python3.10/site-packages/drone_uwb/integration/gazebo/gazebo_rig.py'))
    package = ModuleType('ament_index_python')
    package.__path__ = []
    packages = ModuleType('ament_index_python.packages')
    requested = []
    share = tmp_path / 'share/drone_uwb'

    def package_share(name):
        requested.append(name)
        return str(share)

    packages.get_package_share_directory = package_share
    monkeypatch.setitem(sys.modules, 'ament_index_python', package)
    monkeypatch.setitem(sys.modules, 'ament_index_python.packages', packages)
    assert rig._default_config_directory() == share / 'config'
    assert requested == ['drone_uwb']
