"""Completion must be durable before a consumer can see END."""
import json
from types import SimpleNamespace
import pytest

pytest.importorskip('rclpy')
from drone_mission.node import FlightNode
from test_mission_chain import rig, sample


@pytest.mark.parametrize('disk_failure', [False, True])
def test_first_terminal_publication_requires_completion_fsync(tmp_path, monkeypatch, disk_failure):
    session = rig()
    session.assignment, session.intent = session.intent, None
    session.home_verified = True
    session.landing_verified = False
    session.completed_waypoints = 2
    session.scan_results = [dict(outcome='SUCCEEDED')]
    session.enter('LANDING', 'fixture', 11.)
    session.accepted_command = True
    session.command_confirmed_s = 11.
    trace = []
    def publish(msg):
        trace.append(('publish', json.loads(msg.data)))
    def fsync(fd):
        trace.append(('fsync', fd))
        if disk_failure:
            raise OSError('injected storage failure')
    monkeypatch.setattr('drone_mission.node.os.fsync', fsync)
    monkeypatch.setattr('drone_mission.node.time.monotonic', lambda: 12.)
    clock = SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=round(12e9)))
    pub = SimpleNamespace(publish=publish)
    with (tmp_path/'events.jsonl').open('w') as log:
        adapter = SimpleNamespace(session=session, settings=session.settings,
            snapshot=lambda: sample(12.), record_fault=False, log=log,
            previous_state='LANDING', samples={}, target_feedback=None,
            get_clock=lambda: clock, status_pub=pub, result_pub=pub,
            get_logger=lambda: SimpleNamespace(info=lambda _: None))
        adapter.record=lambda event: FlightNode.record(adapter, event)
        FlightNode.tick(adapter)
    assert trace[0][0] == 'fsync'
    statuses = [item[1] for item in trace if item[0]=='publish']
    assert len(statuses)==2
    assert all(value['state']==('FAILED' if disk_failure else 'END') for value in statuses)
    assert all(value['mission_complete'] is (not disk_failure) for value in statuses)
