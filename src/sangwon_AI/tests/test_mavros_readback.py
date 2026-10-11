"""The diagnostic ROS transport cannot carry parameter writes or flight commands."""
import importlib.util
from pathlib import Path
import pytest
from types import SimpleNamespace

spec = importlib.util.spec_from_file_location(
    'px4_mavros_readback', Path(__file__).parents[1] / 'ops/px4_mavros_readback.py')
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)
QUERIES = ('param show MIS_TAKEOFF_ALT', 'listener estimator_aid_src_ev_pos -n 1')


def packet(text, **changes):
    data = list(text.encode('ascii'))
    args = dict(device=10, flags=6, timeout=0, baudrate=0, count=len(data),
                data=data+[0]*(70-len(data)), queries=QUERIES)
    args.update(changes)
    return module.allowed_shell_packet(**args)


@pytest.mark.parametrize('query', QUERIES)
def test_exact_reads_only(query):
    assert packet(query+'\n')


@pytest.mark.parametrize('text', [
    'commander arm\n', 'param set MIS_TAKEOFF_ALT 2\n',
    'param show MIS_TAKEOFF_ALT; commander arm\n',
    'listener estimator_aid_src_ev_pos -n 1\ncommander arm\n',
    'param show MIS_TAKEOFF_ALT\r\n',
])
def test_reject_writes_and_appended_commands(text):
    assert not packet(text)


@pytest.mark.parametrize('change', [dict(device=0), dict(flags=7), dict(timeout=1),
                                   dict(baudrate=1), dict(count=0), dict(data=[0]*69)])
def test_reject_other_packet_shapes(change):
    assert not packet(QUERIES[0]+'\n', **change)


def test_release_has_no_payload():
    assert packet('', flags=0)
    assert not packet('commander arm\n', flags=0, count=0)


@pytest.mark.parametrize('counts,expected', [([0,0,1],True),([2,2,2],False)])
def test_heartbeat_waits_for_one_discovered_router(monkeypatch,counts,expected):
    transport=module.MavrosReadback.__new__(module.MavrosReadback)
    transport.heartbeat=SimpleNamespace(base_mode=0)
    transport.state=SimpleNamespace(connected=True,armed=False)
    transport.decode_error=None
    clock=[0.]
    calls=[]
    transport.publisher=SimpleNamespace(get_subscription_count=lambda:counts[min(len(calls)-1,len(counts)-1)])
    def receive(**kwargs):
        calls.append(1)
        clock[0]+=.1
    transport.recv_match=receive
    monkeypatch.setattr(module.time,'monotonic',lambda:clock[0])
    assert (transport.wait_heartbeat(.3) is transport.heartbeat) is expected
    assert len(calls)==3


@pytest.mark.parametrize('change', [dict(state_at=0), dict(heartbeat_at=0),
    dict(state=SimpleNamespace(connected=False, armed=False)),
    dict(state=SimpleNamespace(connected=True, armed=True)),
    dict(heartbeat=SimpleNamespace(base_mode=128)), dict(state=None)])
def test_no_query_when_fc_stale_disconnected_or_armed(monkeypatch, change):
    transport = module.MavrosReadback.__new__(module.MavrosReadback)
    transport.queries = QUERIES
    transport.state = SimpleNamespace(connected=True, armed=False)
    transport.heartbeat = SimpleNamespace(base_mode=0)
    transport.state_at = transport.heartbeat_at = 10
    for name, value in change.items():
        setattr(transport, name, value)
    monkeypatch.setattr(module.time, 'monotonic', lambda: 11)
    data = list((QUERIES[0]+'\n').encode('ascii'))
    # No publisher/encoder is available; refusal must precede any outbound work.
    with pytest.raises(RuntimeError, match='Fresh connected and disarmed'):
        transport.serial_control_send(10, 6, 0, 0, len(data), data+[0]*(70-len(data)))
