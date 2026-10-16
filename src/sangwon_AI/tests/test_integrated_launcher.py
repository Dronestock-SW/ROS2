"""Role selection must retain telemetry when a shared observation bridge exists."""
import importlib.util
from pathlib import Path
import pytest


def load(name):
    path = Path(__file__).parents[1] / 'ops' / (name+'.py')
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


@pytest.mark.parametrize('cpp,external,names', [
    (False,False,['web','uwb_bridge','mission','telemetry']),
    (False,True,['web','mission','telemetry']),
    (True,False,['web','uwb_bridge','observer','telemetry']),
    (True,True,['web','observer','telemetry']),
])
def test_execution_selection_preserves_web_and_telemetry(cpp, external, names):
    commands = [(name,[name]) for name in ('web','uwb_bridge','mission','telemetry')]
    result = load('final_field_session').select_execution_commands(
        commands, ['cpp-observer'] if cpp else None, external)
    assert [name for name,_ in result] == names
    assert result[-1] == ('telemetry',['telemetry'])
    if cpp:
        assert dict(result)['observer'] == ['cpp-observer']


def test_timing_queries_contain_only_fixed_read_operations():
    queries = load('px4_sensor_readback').TIMING_QUERIES
    assert 'param show COM_OF_LOSS_T' in queries
    assert 'param show EKF2_EV_DELAY' in queries
    assert 'param show EKF2_NOAID_TOUT' in queries
    assert all(q == 'ver all' or q.startswith(('param show ','listener ')) for q in queries)
    assert not any('param set' in q or 'commander' in q for q in queries)
