"""Inspect real launch actions without starting nodes or contacting hardware."""
import importlib.util
from pathlib import Path
import pytest

pytest.importorskip('launch_ros')
from launch import LaunchContext
from launch.actions import DeclareLaunchArgument
from launch.utilities import perform_substitutions


@pytest.mark.parametrize('ground_only', [False, True])
def test_bridge_scope_is_independent_of_mission_authority(monkeypatch, ground_only):
    path = Path(__file__).resolve().parents[1] / 'launch/test_flight.launch.py'
    spec = importlib.util.spec_from_file_location('flight_launch', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    original = module.Node
    rows = []

    def capture(**kwargs):
        rows.append(kwargs)
        return original(**kwargs)

    monkeypatch.setattr(module, 'Node', capture)
    bridge_settings = []
    for execute in ('false', 'true'):
        rows.clear()
        context = LaunchContext()
        for entity in module.generate_launch_description().entities:
            if isinstance(entity, DeclareLaunchArgument):
                context.launch_configurations[entity.name] = perform_substitutions(context, entity.default_value)
        assert context.launch_configurations['bridge_ground_only'] == 'false'
        context.launch_configurations.update(tag='B', bridge_enabled='true',
            bridge_ground_only=str(ground_only).lower(), execute=execute)
        module.components(context)
        settings = next(r['parameters'][0] for r in rows if r['executable'] == 'uwb_px4_bridge')
        bridge_settings.append(settings)
        assert settings['ground_only'] is ground_only
        assert settings['enabled'] is True
        assert settings['alignment_confirmed'] is False
        assert settings['timing_confirmed'] is False
        mission = next(r['parameters'][0] for r in rows if r['executable'] == 'flight_mission')
        assert mission['execute'] is (execute == 'true')
    assert bridge_settings[0] == bridge_settings[1]
