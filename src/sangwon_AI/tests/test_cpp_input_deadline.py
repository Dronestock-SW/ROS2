"""Encoding time consumes the decision I/O budget; input faults stay visible."""
import importlib.util
from pathlib import Path
import pytest

path=Path(__file__).parents[2]/'drone_mission/drone_mission/cpp_mission_ai.py'
spec=importlib.util.spec_from_file_location('cpp_input_deadline',path)
module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)


def test_encoding_delay_exhausts_budget_before_any_pipe_access(monkeypatch):
    peer=module.CppMissionAI.__new__(module.CppMissionAI)
    peer.sequence=0;peer.session_id='test';clock=iter((10.,10.101))
    monkeypatch.setattr(module.time,'monotonic',lambda:next(clock))
    # There is deliberately no process: an exhausted request must not write.
    with pytest.raises(ValueError,match='request_encoding_timeout'):
        peer._exchange('step',{})


def test_lidar_time_fault_is_preserved_for_cpp_diagnostics():
    class Sample:
        lidar_input_error='attitude_not_bracketed'
        def __getattr__(self,key):return None
    row=module.state_input(Sample(),0.)
    assert row['lidar_input_error']=='attitude_not_bracketed'
