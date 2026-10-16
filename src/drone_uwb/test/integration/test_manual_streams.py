"""Command acceptance/retry boundaries with fake ROS services, no DDS or FC."""
from concurrent.futures import Future
import sys
from types import ModuleType, SimpleNamespace

from drone_uwb.integration.ros import manual_streams as streams


def test_interval_attempt_budget_and_backoff():
    requests = streams.IntervalRequests(((132, 50.), (31, 100.)))
    assert requests.begin(0) == (132, 50.)
    assert requests.finish(False, 0) == 'retry'
    assert requests.begin(.999) is None
    assert requests.begin(1) == (132, 50.)
    assert requests.finish(False, 1) == 'retry'
    assert requests.begin(2) == (132, 50.)
    assert requests.finish(False, 2) == 'exhausted'
    assert requests.begin(2) == (31, 100.)
    assert requests.finish(True, 2) == 'accepted'
    assert requests.begin(3) is None
    requests.reset()
    assert requests.begin(0) == (132, 50.)


def run_node(monkeypatch, scenario):
    clock = [0.]
    monkeypatch.setattr(streams.time, 'monotonic', lambda: clock[0])
    calls, removed, logs = [], [], []
    class Client:
        ready = True
        def service_is_ready(self): return self.ready
        def call_async(self, request):
            future = Future()
            calls.append((request, future))
            return future
        def remove_pending_request(self, future): removed.append(future)
    client = Client()
    class Node:
        def __init__(self, name): self.name = name
        def create_client(self, *args): return client
        def create_subscription(self, *args): pass
        def create_timer(self, *args): pass
        def get_logger(self): return SimpleNamespace(info=logs.append, warning=logs.append)
        def destroy_node(self): pass
    modules = {
        'rclpy': dict(init=lambda: None, spin=lambda node: scenario(node, clock, calls, removed, logs),
                      ok=lambda: True, shutdown=lambda: None),
        'rclpy.node': dict(Node=Node),
        'rclpy.qos': dict(qos_profile_sensor_data=None),
        'rclpy.executors': dict(ExternalShutdownException=type('Shutdown', (Exception,), {})),
        'mavros_msgs': {}, 'mavros_msgs.msg': dict(State=object),
        'mavros_msgs.srv': dict(CommandLong=SimpleNamespace(Request=lambda **kw: SimpleNamespace(**kw))),
    }
    for name, contents in modules.items():
        module = ModuleType(name)
        module.__dict__.update(contents)
        monkeypatch.setitem(sys.modules, name, module)
    streams.main()
    return calls


def state(connected=True, armed=False):
    return SimpleNamespace(connected=connected, armed=armed)


def test_success_false_default_result_zero_is_not_an_ack(monkeypatch):
    def scenario(node, clock, calls, removed, logs):
        node.receive(state()); node.tick()
        calls[-1][1].set_result(SimpleNamespace(success=False, result=0))
        node.tick()
        assert node.requests.index == 0 and node.requests.attempts == 1
        node.tick(); assert len(calls) == 1
        clock[0] = 1.; node.receive(state()); node.tick()
        assert len(calls) == 2 and calls[-1][0].param1 == calls[0][0].param1
        calls[-1][1].set_result(SimpleNamespace(success=True, result=0)); node.tick()
        assert node.requests.index == 1 and 'actual rate unverified' in logs[-1]
    run_node(monkeypatch, scenario)


def test_timeout_disconnect_and_flight_boundaries(monkeypatch):
    def scenario(node, clock, calls, removed, logs):
        node.receive(state(armed=True)); node.tick(); assert not calls
        node.receive(state()); clock[0] = 1.6; node.tick(); assert not calls
        node.receive(state()); node.tick(); assert len(calls) == 1
        pending = calls[-1][1]
        clock[0] += 3.01; node.tick()
        assert pending.cancelled() and removed == [pending]
        clock[0] += 1; node.receive(state()); node.tick(); assert len(calls) == 2
        pending = calls[-1][1]
        node.receive(state(connected=False)); node.tick()
        assert pending.cancelled() and node.requests.index == 0 and node.requests.attempts == 0
        node.receive(state()); node.tick(); assert calls[-1][0].param1 == streams.RATES[0][0]
    run_node(monkeypatch, scenario)


def test_serialized_requests_distance_sensor_50hz_only_command_511(monkeypatch):
    def scenario(node, clock, calls, removed, logs):
        node.receive(state())
        for _ in streams.RATES:
            node.tick()
            pending = calls[-1][1]
            count = len(calls)
            node.tick(); assert len(calls) == count  # no concurrent requests
            pending.set_result(SimpleNamespace(success=True, result=0)); node.tick()
        node.tick(); assert len(calls) == len(streams.RATES)
    calls = run_node(monkeypatch, scenario)
    assert all(request.command == 511 for request, _ in calls)
    distance = [request for request, _ in calls if request.param1 == 132]
    assert len(distance) == 1 and distance[0].param2 == 20000


def test_rejected_ack_and_exception_exhaust_same_message(monkeypatch):
    def scenario(node, clock, calls, removed, logs):
        for attempt in range(3):
            node.receive(state()); node.tick()
            if attempt == 1:
                calls[-1][1].set_exception(RuntimeError('transport'))
            else:
                calls[-1][1].set_result(SimpleNamespace(success=True, result=2))
            node.tick()
            clock[0] += 1
        assert [r.param1 for r, _ in calls] == [streams.RATES[0][0]] * 3
        assert node.requests.index == 1 and 'exhausted' in logs[-1]
    run_node(monkeypatch, scenario)
